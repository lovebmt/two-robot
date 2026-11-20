"""Wizard hiệu chuẩn (đi thẳng, quay, khoảng cách ToF) và kiểm tra trước khi chạy."""
import asyncio
import math

import pytest

from tworobot import calibration as CAL
from tworobot import params as P
from tworobot.sim import batch
from tworobot.sim.engine import SimEngine


def _run_prim(eng, r, prim, **args):
    ev = []
    r.fw.emit = ev.append
    t0 = dict(x=r.body.x, y=r.body.y, yaw=r.body.th_acc)
    res = r.fw.handle_cmd({"op": "prim", "id": 7, "prim": prim, **args})
    assert res["ok"], res
    for _ in range(400):
        eng.advance(0.25)
        done = [e for e in ev if e.get("e") == "prim_done"]
        if done:
            eng.advance(0.3)
            return done[-1], t0
    raise AssertionError(f"{prim} không kết thúc")


def _robot(mm, seed=1):
    eng = SimEngine(6000, 3000, seed=seed)
    r = eng.add_robot("a", "a", 800, 1500, 0.0, mm)
    eng.advance(3.0)  # auto calib IMU
    assert r.fw.imu_ok
    return eng, r


def test_drive_calibration_recovers_wheel_diameter():
    eng, r = _robot({"diam_err_l": 0.03, "diam_err_r": 0.03})  # bánh thật to hơn 3 %
    done, t0 = _run_prim(eng, r, "CAL_DRIVE", mm=1000)
    assert done["ok"]
    measured = math.hypot(r.body.x - t0["x"], r.body.y - t0["y"])  # "thước" trong sim
    patch, info = CAL.drive(r.fw.params.active, done["result"], measured)
    assert abs(patch["wheel.diameter_mm"] - 65 * 1.03) < 0.4
    assert info["err_before_pct"] < -2.0  # trước hiệu chuẩn encoder đo thiếu ~3 %


def test_turn_calibration_recovers_gyro_scale():
    eng, r = _robot({"gyro_scale_err": 0.02, "slip_turn": 0.03})
    done, t0 = _run_prim(eng, r, "CAL_TURN", deg=360)
    assert done["ok"]
    measured = math.degrees(r.body.th_acc - t0["yaw"])
    assert abs(measured - 360) > 3  # chưa hiệu chuẩn: quay thiếu ~7°
    patch, info = CAL.turn(r.fw.params.active, done["result"], measured)
    assert abs(patch["imu.gyro_scale_z"] - 1 / 1.02) < 0.004
    # track hiệu dụng lớn hơn track hình học vì trượt bánh khi quay
    assert patch["wheel.track_eff_mm"] > 130


def test_calibration_math_rejects_bad_input():
    cfg = P.defaults()
    with pytest.raises(CAL.CalError):
        CAL.drive(cfg, {"enc_mm": 1000}, 600)          # nhập nhầm đơn vị
    with pytest.raises(CAL.CalError):
        CAL.turn(cfg, {"est_deg": 360, "arc_l_mm": -200, "arc_r_mm": 200}, 20)
    p, _ = CAL.gap(cfg, {"dl_mm": 104, "dr_mm": 98, "offset_l_mm": 0, "offset_r_mm": 0}, 100, keep_diff=True)
    assert p == {"range.offset_l_mm": -1.0, "range.offset_r_mm": -1.0}
    p, _ = CAL.gap(cfg, {"dl_mm": 104, "dr_mm": 98, "offset_l_mm": 0, "offset_r_mm": 0}, 100, keep_diff=False)
    assert p == {"range.offset_l_mm": -4.0, "range.offset_r_mm": 2.0}


@pytest.mark.parametrize("offset,expect_ok,expect_text", [
    ({}, True, None),
    ({"follower": {"dyaw": 3}}, True, None),                  # lệch góc nhỏ → tự ALIGN rồi chạy
    ({"follower": {"dyaw": 10}}, False, "thẳng hàng"),         # lệch góc lớn → bắt đặt lại
    ({"follower": {"dx": -150}}, False, "Khoảng cách"),       # xe sau quá xa
    ({"follower": {"dy": 45}}, True, None),                   # lệch ngang 45 mm: chùm 25° vẫn thấy tấm → KHÔNG phát hiện được
    ({"follower": {"dy": 90}}, False, "2 mắt"),               # lệch ngang lớn → 1 mắt mất tấm / 2 mắt chênh
])
def test_preflight_gate(offset, expect_ok, expect_text):
    r = asyncio.run(batch.run_scenario(batch.PRESETS["L"], seed=2, idle_before_s=3, start_offset=offset))
    assert r["ok"] == expect_ok, r.get("error")
    if expect_text:
        assert "Kiểm tra trước khi chạy" in r["error"] and expect_text in r["error"]
    assert r["collisions"] == 0


def test_encoder_hand_rotation_calibration():
    cfg = P.defaults()  # PPR 11, tỉ số 34 → 1496 tick/vòng
    patch, info = CAL.encoder(cfg, "l", 2 * 11 * 4 * 46.8, 2)       # quay tới 2 vòng, motor thật 1:46.8
    assert abs(patch["enc.gear_ratio"] - 46.8) < 0.01 and info["direction_ok"] and "enc.invert_l" not in patch
    assert info["nearest_standard"] == 46.8
    patch, info = CAL.encoder(cfg, "r", -2 * 11 * 4 * 47.0, 2, other={"gear_ratio": 46.8})  # đếm âm → đảo chiều
    assert patch["enc.invert_r"] is True and abs(patch["enc.gear_ratio"] - 46.9) < 0.01
    with pytest.raises(CAL.CalError):
        CAL.encoder(cfg, "r", 3, 2)  # encoder không có tín hiệu

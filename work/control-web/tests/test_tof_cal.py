"""Hiệu chuẩn 2 mắt ToF lắp lệch (quét ±40° trước mặt phẳng)."""
import asyncio
import math

import pytest

from tworobot.sim import batch
from tworobot.sim.engine import SimEngine

BAD_MOUNT = {"tof_dx_l": 1.5, "tof_off_l": -5.0, "tof_off_r": 7.0, "tof_yaw_l": 3.0, "tof_yaw_r": -2.0}
C_TRUE = 1.5 + (7.0 - (-5.0))  # (dx_l − dx_r) + (off_r − off_l)


def _calibrate(mm, yaw_deg=4.0, dist=200, seed=7):
    eng = SimEngine(3000, 1800, seed=seed)
    r = eng.add_robot("a", "a", 3000 + eng.world.wall_margin - 120 - dist, 900, math.radians(yaw_deg), mm)
    ev = []
    r.fw.emit = ev.append
    eng.advance(3.0)  # auto calib IMU
    r.fw.handle_ws({"t": "cmd", "op": "prim", "id": 1, "prim": "TOF_CAL"})
    for _ in range(120):
        eng.advance(0.5)
        done = [e for e in ev if e.get("e") == "prim_done"]
        if done:
            return done[-1], r
    raise AssertionError("TOF_CAL không kết thúc")


@pytest.mark.parametrize("yaw,dist", [(-8, 150), (0, 250), (6, 200)])
def test_tof_cal_recovers_mounting_offset(yaw, dist):
    d, r = _calibrate(BAD_MOUNT, yaw, dist, seed=yaw + dist)
    assert d["ok"], d
    res = d["result"]
    assert abs(res["c_mm"] - C_TRUE) < 2.0
    assert abs(res["theta0_deg"] + yaw) < 2.0
    assert r.fw.state == "IDLE"  # bài hiệu chuẩn chạy từ IDLE và trả về IDLE


def test_tof_cal_requires_imu_calibrated():
    eng = SimEngine(3000, 1800, seed=1)
    r = eng.add_robot("a", "a", 3000, 900, 0.0)
    r.fw.params.submit({"imu.auto_calib": False}, running=False)
    eng.advance(0.1)
    assert r.fw.handle_cmd({"op": "prim", "id": 1, "prim": "TOF_CAL"}) == {"ok": False, "err": "imu_uncal", "state": "IDLE"}


def test_bad_mount_breaks_following_and_calibration_fixes_it():
    mm = {"leader": BAD_MOUNT, "follower": BAD_MOUNT}
    bad = asyncio.run(batch.run_scenario(batch.PRESETS["L"], seed=2, mismatch=mm, idle_before_s=5))
    assert not bad["ok"]  # Δψ lệch ~13° → mất leader
    d, _ = _calibrate(BAD_MOUNT)
    res = d["result"]
    fix = {"range.offset_l_mm": res["offset_l_mm"], "range.offset_r_mm": res["offset_r_mm"]}
    good = asyncio.run(batch.run_scenario(batch.PRESETS["L"], seed=2, mismatch=mm, robot_patch=fix, idle_before_s=5))
    assert good["ok"] and good["collisions"] == 0, good.get("error")

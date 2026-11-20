"""Gyro MPU6050: calib khi đứng yên, ZUPT, bù nhiệt (wiki/algorithms.md §2.1)."""
import math

from tworobot.sim.engine import SimEngine

FAST_WARM = {"gyro_bias_dps": 2.0, "gyro_warm_dps": 0.8, "gyro_warm_tau_s": 60.0, "gyro_rw_dps": 0.0}


def _robot(patch=None, mismatch=None, seed=1):
    eng = SimEngine(3000, 2000, seed=seed)
    r = eng.add_robot("a", "a", 1000, 1000, 0.0, {**FAST_WARM, **(mismatch or {})})
    events = []
    r.fw.emit = events.append
    if patch:
        assert r.fw.params.submit(patch, running=False)["ok"]
    return eng, r, events


def _calib_events(events):
    return [e for e in events if e.get("e") == "calib_done"]


def test_auto_calib_estimates_bias_and_arm_gate():
    eng, r, events = _robot({"imu.auto_calib": False})
    r.fw.role = "leader"
    r.fw.path = {"x": 1}
    r.fw.handle_ws({"t": "cmd", "op": "arm", "seq": 1})
    assert events[-1]["t"] == "nack" and events[-1]["err"] == "imu_uncal"
    r.fw.handle_ws({"t": "cmd", "op": "calib_imu", "seq": 2})
    eng.advance(3.0)
    done = _calib_events(events)
    assert done and done[-1]["ok"]
    truth = math.degrees(r.body.gyro_bias_true())
    assert abs(done[-1]["result"]["bias_dps"] - truth) < 0.05
    r.fw.handle_ws({"t": "cmd", "op": "arm", "seq": 3})
    assert events[-1]["t"] == "ack"


def test_bump_during_calib_restarts():
    eng, r, events = _robot({"imu.auto_calib": False})
    r.fw.handle_ws({"t": "cmd", "op": "calib_imu"})
    eng.advance(1.0)
    eng.inject("bump", "a", dur=0.3, rate_dps=20)
    eng.advance(4.0)
    done = _calib_events(events)
    assert done and done[-1]["ok"] and done[-1]["result"]["restarts"] >= 1


def test_no_heading_integration_before_calib():
    """Chưa calib + đứng yên: bias chưa biết → không được tích phân hướng."""
    eng, r, _ = _robot({"imu.auto_calib": False})
    eng.advance(10.0)
    assert r.fw.heading.psi == 0.0


def test_warmup_drift_needs_zupt():
    # Chỉ calib 1 lần lúc bật nguồn, tắt ZUPT + bù nhiệt: chip nóng dần → bias đổi → hướng trôi.
    eng, r, _ = _robot({"imu.zupt_enable": False, "imu.temp_comp": False})
    eng.advance(60.0)
    assert abs(math.degrees(r.fw.heading.psi)) > 3
    # Mặc định (calib + ZUPT): đứng yên thì hướng gần như không đổi.
    eng, r, _ = _robot()
    eng.advance(60.0)
    assert abs(math.degrees(r.fw.heading.psi)) < 0.5


def test_temperature_coefficient_learned_while_idle():
    eng, r, _ = _robot()
    eng.advance(120.0)  # vừa bật nguồn, chip nóng dần
    h = r.fw.heading
    assert h.k_temp is not None
    k_true = FAST_WARM["gyro_warm_dps"] / 8.0  # temp_rise_c mặc định 8 °C
    assert abs(math.degrees(h.k_temp) - k_true) < 0.25 * k_true

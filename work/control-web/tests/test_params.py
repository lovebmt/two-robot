import math

from tworobot import params as P
from tworobot.fw.paramsvc import ParamService


class MemHal:
    def __init__(self):
        self.cfg = None
        self.t = 0

    def now_ms(self):
        return self.t

    def storage_load(self):
        return self.cfg

    def storage_save(self, data):
        self.cfg = data
        return True

    def nvs_load(self):
        return {}

    def nvs_save(self, d):
        pass


def test_defaults_pass_validation():
    new, errors = P.validate_patch(P.defaults(), {})
    assert errors == {}


def test_range_and_type_errors():
    _, errors = P.validate_patch(P.defaults(), {"wheel.diameter_mm": 5, "body.width_mm": "abc", "x.y": 1})
    assert set(errors) == {"wheel.diameter_mm", "body.width_mm", "x.y"}


def test_cross_check_sensor_outside_body():
    _, errors = P.validate_patch(P.defaults(), {"range.x_mm": 200})
    assert "range.x_mm" in errors


def test_derived_mm_per_tick():
    cfg = P.defaults()
    d = P.derive_robot(cfg)
    expected = math.pi * 65 / (11 * 4 * cfg["enc.gear_ratio"])
    assert abs(d["mm_per_tick_l"] - expected) < 1e-9
    assert abs(d["r_sweep_mm"] - math.hypot(120, 60)) < 1e-9


def test_formation_geometry():
    c = P.defaults()
    form, errors, _ = P.formation(c, c, P.shared_defaults())
    assert errors == {}
    assert form["g_hold_min"] == math.hypot(120, 60) + 120 + 30
    assert form["exit_dist"] >= form["exit_dist_min"]
    sh = P.shared_defaults()
    sh["gap.target_mm"] = 200
    _, errors, _ = P.formation(c, c, sh)
    assert "gap.target_mm" in errors


def test_paramservice_swap_and_idle_lock():
    hal = MemHal()
    ps = ParamService(hal)
    assert "CFG_DEFAULT" in ps.flags
    res = ps.submit({"pid.wheel_kp": 0.02}, running=True)
    assert res["ok"]
    assert ps.active["pid.wheel_kp"] == 0.01  # chưa swap
    groups = ps.swap_if_pending()
    assert groups == {"pid"} and ps.active["pid.wheel_kp"] == 0.02
    res = ps.submit({"wheel.diameter_mm": 70}, running=True)
    assert not res["ok"] and res["busy"]
    res = ps.submit({"wheel.diameter_mm": 70}, running=False)
    ps.swap_if_pending()
    assert abs(ps.derived["mm_per_tick_l"] - math.pi * 70 / (11 * 4 * ps.active["enc.gear_ratio"])) < 1e-9


def test_paramservice_save_and_reload():
    hal = MemHal()
    ps = ParamService(hal)
    ps.submit({"gap.target_mm": 400}, running=False)
    ps.save()
    assert "CFG_DEFAULT" not in ps.flags
    ps2 = ParamService(hal)
    assert ps2.active["gap.target_mm"] == 400
    assert not ps2.dirty()

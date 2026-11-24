from tworobot import params as P
from tworobot.fw.tof_guard import TofAnomalyGuard


def _armed():
    cfg = P.defaults()
    guard = TofAnomalyGuard()
    for _ in range(3):
        assert guard.update(300, 0, 302, 0, cfg) is False
    return guard, cfg


def test_gradual_change_does_not_trip():
    guard, cfg = _armed()
    for mm in (280, 260, 240, 220, 200):
        assert guard.update(mm, 0, mm + 2, 0, cfg) is False


def test_not_armed_until_both_eyes_have_three_samples():
    cfg = P.defaults()
    guard = TofAnomalyGuard()
    for _ in range(3):
        assert guard.update(300, 0, 8190, 4, cfg) is False
    assert guard.update(15, 0, 8190, 4, cfg) is False


def test_one_eye_sudden_close_reading_trips_after_two_samples():
    guard, cfg = _armed()
    assert guard.update(170, 0, 302, 0, cfg) is False
    assert guard.update(170, 0, 302, 0, cfg) is True


def test_both_eyes_sudden_close_reading_trips():
    guard, cfg = _armed()
    assert guard.update(170, 0, 172, 0, cfg) is False
    assert guard.update(170, 0, 172, 0, cfg) is True


def test_one_eye_loss_trips():
    guard, cfg = _armed()
    assert guard.update(300, 0, 8190, 4, cfg) is False
    assert guard.update(300, 0, 8190, 4, cfg) is True


def test_both_eyes_loss_trips():
    guard, cfg = _armed()
    assert guard.update(8190, 4, 8190, 4, cfg) is False
    assert guard.update(8190, 4, 8190, 4, cfg) is True

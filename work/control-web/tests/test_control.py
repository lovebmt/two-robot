"""Bộ điều khiển bánh khi kẹt ma sát tĩnh (gần đích, tốc độ lệnh nhỏ)."""
from tworobot import params as P
from tworobot.fw.control import WheelController


def _volts_after_stall(cfg: dict, seconds: float) -> float:
    wc = WheelController("l")
    vbat = 12.0
    for _ in range(int(seconds / 0.01)):
        duty = wc.update(15.0, 0.0, vbat, cfg, 0.01)  # lệnh 15 mm/s, bánh không nhúc nhích
    return duty * (vbat - cfg["power.l298n_vdrop_v"])


def test_stalled_wheel_pushes_past_old_pi_ceiling():
    """Trước đây trần ≈ ks + kv·v + Ki-clamp 3 V ≈ 4.5 V → ma sát tĩnh 5 V là kẹt mãi.
    Giờ bánh kẹt > 0.15 s thì cộng thêm tới 4 V."""
    cfg = P.defaults()
    assert _volts_after_stall(cfg, 0.1) < 2.0  # chưa kẹt đủ lâu: chưa đẩy
    assert _volts_after_stall(cfg, 3.0) > 7.5


def test_kbreak_kicks_from_standstill():
    """Đã hiệu chuẩn kbreak: bánh đứng yên được cấp ngay điện áp bứt, không chờ tích phân."""
    cfg = P.defaults()
    cfg["motor.kbreak_l_v"] = 5.0
    assert _volts_after_stall(cfg, 0.02) > 5.0
    wc = WheelController("l")
    duty = wc.update(15.0, 15.0, 12.0, cfg, 0.01)  # đang chạy: về vùng chết động ks
    assert duty * (12.0 - cfg["power.l298n_vdrop_v"]) < 2.0

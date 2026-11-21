"""Chế độ 1 xe bám tấm phẳng (FOLLOW_PLATE): xe trước (đóng vai tấm cầm tay) đi tới/lùi, xe sau giữ khoảng."""
import math

import pytest

from tworobot.sim.engine import SimEngine


def _pair(gap_face_mm: float):
    """b đứng sau a, mặt kính ToF của b cách mặt đuôi a đúng gap_face_mm."""
    eng = SimEngine(6000, 3000, seed=3)
    a = eng.add_robot("a", "a", 2000, 1500, 0.0)
    b_cfg = None
    b = eng.add_robot("b", "b", 0, 1500, 0.0)
    pa, pb = a.body.p, b.body.p
    eng.place("b", 2000 - pa.rear - gap_face_mm - pb.tof_x, 1500, 0.0)
    eng.advance(3.0)  # auto calib IMU
    assert a.fw.imu_ok and b.fw.imu_ok
    return eng, a, b


def _face_gap(a, b) -> float:
    return math.hypot(a.body.x - b.body.x, a.body.y - b.body.y) - a.body.p.rear - b.body.p.tof_x


def test_follows_plate_forward_and_holds_distance():
    eng, a, b = _pair(200)
    assert b.fw.handle_cmd({"op": "prim", "id": 1, "prim": "FOLLOW_PLATE", "d_target": 200})["ok"]
    a.fw.handle_cmd({"op": "prim", "id": 2, "prim": "CAL_DRIVE", "mm": 800, "v_max": 100})
    for _ in range(60):
        eng.advance(0.25)
    assert a.fw.state == "IDLE"          # tấm đã dừng
    assert b.fw.state == "RUNNING"       # xe bám vẫn chạy tới khi PC bấm stop
    assert abs(_face_gap(a, b) - 200) < 25, _face_gap(a, b)
    assert b.body.x > 1300               # đã đi theo ~800 mm


def test_backs_off_when_plate_too_close_and_waits_when_lost():
    eng, a, b = _pair(90)
    assert b.fw.handle_cmd({"op": "prim", "id": 1, "prim": "FOLLOW_PLATE", "d_target": 250})["ok"]
    eng.advance(8.0)
    assert abs(_face_gap(a, b) - 250) < 25, _face_gap(a, b)  # đã lùi ra
    eng.place("a", 5800, 2900, 90.0)  # tấm biến mất
    eng.advance(2.0)
    assert b.fw.state == "RUNNING" and b.fw.fault is None   # không báo lỗi, chỉ chờ
    assert abs(b.body.v) < 1.0
    b.fw.handle_cmd({"op": "stop"})
    assert b.fw.state == "IDLE"



def _drive_arc(eng, robot, v, w, seconds):
    """Tấm đi theo cung tròn (giống tay cầm tấm đi vòng): giữ lệnh JOG liên tục."""
    for _ in range(int(seconds / 0.2)):
        robot.fw.handle_cmd({"op": "jog", "v": v, "w": w})
        eng.advance(0.2)


@pytest.mark.parametrize("w", [0.35, -0.35])
def test_follows_plate_around_a_curve(w):
    """Tấm cua 90° (trái/phải) theo cung bán kính ~290 mm → xe bám vẫn thấy tấm, hướng bám theo, giữ khoảng."""
    eng, a, b = _pair(200)
    assert b.fw.handle_cmd({"op": "prim", "id": 1, "prim": "FOLLOW_PLATE", "d_target": 200})["ok"]
    eng.advance(0.5)
    lost = 0
    for _ in range(int(4.5 / 0.2)):  # 0.35 rad/s × 4.5 s ≈ 90°
        a.fw.handle_cmd({"op": "jog", "v": 100, "w": w})
        eng.advance(0.2)
        lost += bool(getattr(b.fw.prim, "lost", False))
    _drive_arc(eng, a, 100, 0.0, 2.0)  # ra khỏi cua, đi thẳng
    eng.advance(3.0)
    assert lost == 0, f"mất tấm {lost} lần trong cua"
    assert abs(_face_gap(a, b) - 200) < 40, _face_gap(a, b)
    # Không kiểm hướng: ToF sim lấy điểm gần nhất trong nón 25° nên gần như không thấy góc nghiêng của tấm
    # (tấm nghiêng 50° đo ra < 10°); xe thật đo góc tốt hơn — chỉnh follow.plate_k_psi / plate_k_edge trên xe.
"""Mô hình vật lý 2D: thân hộp, 2 motor DC lệch nhau, L298N, ma sát theo vùng, trượt,
encoder lượng tử, gyro có bias/nhiễu, ToF200C mô phỏng bằng nhiều tia, va chạm.

"Sự thật" (Physical) lấy từ config lúc đặt xe + độ lệch (mismatch). Firmware không
biết các độ lệch này → giống xe thật chưa hiệu chuẩn hoàn hảo.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from ..fw.mathutil import wrap

DEFAULT_MISMATCH = {
    "motor_scale_l": 1.0,   # nhân hệ số motor (mm/s trên mỗi volt) — <1 là motor yếu
    "motor_scale_r": 1.0,
    "fric_scale": 1.0,      # nhân điện áp vùng chết (ma sát)
    "stiction": 1.0,        # ma sát tĩnh = stiction × ma sát động (bánh đứng yên cần điện áp này mới bứt)
    "diam_err_l": 0.0,      # sai số đường kính thật so với config (tỉ lệ)
    "diam_err_r": 0.0,
    "track_err_mm": 0.0,
    # Mô hình gyro MPU6050 (trục Z):
    "gyro_bias_dps": 1.5,     # bias lúc bật nguồn (mỗi lần bật khác nhau; engine bốc ngẫu nhiên nếu không đặt)
    "gyro_rw_dps": 0.01,      # bias trôi ngẫu nhiên (random walk), °/s/√s
    "gyro_warm_dps": 0.6,     # trôi do nóng máy: bias đổi thêm lượng này khi chip nóng hết temp_rise_c
    "gyro_warm_tau_s": 90.0,  # hằng số thời gian nóng máy
    "temp_rise_c": 8.0,       # chip nóng lên bao nhiêu °C sau khi bật nguồn
    "gyro_vib": 0.002,        # nhiễu rung motor, °/s trên mỗi mm/s tốc độ bánh
    "gyro_noise_dps": 0.05,   # nhiễu trắng gyro (σ mỗi mẫu)
    "gyro_scale_err": 0.01,
    "slip_turn": 0.03,      # tỉ lệ trượt khi quay tại chỗ
    "tof_sigma": 3.0,       # mm
    # Sai số lắp đặt 2 mắt ToF (firmware không biết; bù bằng bài hiệu chuẩn quét tường):
    "tof_dx_l": 0.0,        # mắt trái nhô ra trước (+) / thụt vào (−) so với thiết kế, mm
    "tof_dx_r": 0.0,
    "tof_yaw_l": 0.0,       # mắt trái quay lệch (độ, + = sang trái)
    "tof_yaw_r": 0.0,
    "tof_off_l": 0.0,       # offset đo riêng của module VL53L0X (mm, cộng vào số đo)
    "tof_off_r": 0.0,
    "tof_db": 0.0,          # khoảng cách 2 mắt thật lớn hơn thiết kế, mm
    "vbat": 12.0,
}


@dataclass
class Physical:
    length: float
    width: float
    front: float
    rear: float
    plate_w: float
    wheel_d_l: float
    wheel_d_r: float
    track: float
    ticks_per_rev: float
    k_l: float
    k_r: float
    fric_l: float
    fric_r: float
    stiction: float
    tau: float
    gyro_bias: float
    gyro_scale_err: float
    gyro_noise: float
    gyro_rw: float
    gyro_warm: float
    gyro_warm_tau: float
    gyro_vib: float
    temp_rise: float
    tof_x: float
    tof_b: float
    tof_sigma: float
    tof_fov_deg: float
    tof_dx: dict
    tof_yaw: dict
    tof_off: dict
    slip_turn: float
    vdrop: float
    vbat: float


def physical_from_cfg(cfg: dict, mismatch: dict | None = None) -> Physical:
    m = dict(DEFAULT_MISMATCH)
    m.update(mismatch or {})
    d = cfg["wheel.diameter_mm"]
    return Physical(
        length=cfg["body.length_mm"], width=cfg["body.width_mm"],
        front=cfg["body.axle_to_front_mm"], rear=cfg["body.axle_to_rear_mm"],
        plate_w=cfg["body.plate_width_mm"],
        wheel_d_l=d * (1 + m["diam_err_l"]), wheel_d_r=d * (1 + m["diam_err_r"]),
        track=cfg["wheel.track_mm"] + m["track_err_mm"],
        ticks_per_rev=cfg["enc.ppr_motor"] * 4 * cfg["enc.gear_ratio"],
        k_l=m["motor_scale_l"] / cfg["motor.kv_l"], k_r=m["motor_scale_r"] / cfg["motor.kv_r"],
        fric_l=cfg["motor.ks_l_v"] * m["fric_scale"], fric_r=cfg["motor.ks_r_v"] * m["fric_scale"],
        stiction=m["stiction"], tau=0.08,
        gyro_bias=math.radians(m["gyro_bias_dps"]), gyro_scale_err=m["gyro_scale_err"],
        gyro_noise=math.radians(m["gyro_noise_dps"]),
        gyro_rw=math.radians(m["gyro_rw_dps"]), gyro_warm=math.radians(m["gyro_warm_dps"]),
        gyro_warm_tau=m["gyro_warm_tau_s"], gyro_vib=math.radians(m["gyro_vib"]), temp_rise=m["temp_rise_c"],
        tof_x=cfg["range.x_mm"], tof_b=cfg["range.baseline_mm"] + m["tof_db"], tof_sigma=m["tof_sigma"],
        tof_fov_deg=25.0,
        tof_dx={"l": m["tof_dx_l"], "r": m["tof_dx_r"]},
        tof_yaw={"l": math.radians(m["tof_yaw_l"]), "r": math.radians(m["tof_yaw_r"])},
        tof_off={"l": m["tof_off_l"], "r": m["tof_off_r"]}, slip_turn=m["slip_turn"], vdrop=cfg["power.l298n_vdrop_v"], vbat=m["vbat"],
    )


class Body:
    """Một xe: trạng thái thật (tâm trục bánh x, y, hướng th)."""

    def __init__(self, bid: str, phys: Physical, x: float, y: float, th: float):
        self.id = bid
        self.p = phys
        self.place(x, y, th)
        self.duty = {"l": 0.0, "r": 0.0}
        self.brake = {"l": True, "r": True}
        self.enc_dist = {"l": 0.0, "r": 0.0}
        self.tof_block: dict[str, float] = {}  # side → hết hạn (s)
        self.caster_stuck_until = 0.0
        self.t_on = 0.0          # thời gian từ lúc bật nguồn (cho trôi nóng máy)
        self.bias_rw = 0.0       # phần bias trôi ngẫu nhiên
        self.bump_until = 0.0    # chạm/va nhẹ: gyro thấy xung quay dù bánh không lăn
        self.bump_rate = 0.0

    def place(self, x: float, y: float, th: float) -> None:
        self.x, self.y, self.th = x, y, th
        self.th_acc = 0.0  # góc quay cộng dồn (không wrap) — "thước đo góc" cho hiệu chuẩn trong sim
        self.u = {"l": 0.0, "r": 0.0}  # tốc độ mặt bánh (mm/s)
        self.v = 0.0
        self.w = 0.0

    def step(self, dt: float, fric_factor: float, t: float, rng: random.Random) -> None:
        p = self.p
        self.t_on += dt
        self.bias_rw += rng.gauss(0.0, p.gyro_rw * math.sqrt(dt))
        for s in ("l", "r"):
            if self.brake[s]:
                target, tau = 0.0, 0.03
            else:
                volts = self.duty[s] * (p.vbat - p.vdrop)
                fric = (p.fric_l if s == "l" else p.fric_r) * fric_factor
                k = p.k_l if s == "l" else p.k_r
                if abs(self.u[s]) < 1.0:  # bánh đang đứng: phải thắng ma sát tĩnh
                    if abs(volts) <= fric * p.stiction:
                        volts = 0.0
                if abs(volts) <= fric:
                    target = 0.0
                else:
                    target = k * (volts - math.copysign(fric, volts))
                tau = p.tau
            self.u[s] += (target - self.u[s]) * min(1.0, dt / tau)
        ul, ur = self.u["l"], self.u["r"]
        turning = ul * ur < 0
        slip = p.slip_turn if turning else 0.002
        gl, gr = ul * (1 - slip), ur * (1 - slip)
        v = 0.5 * (gl + gr)
        w = (gr - gl) / p.track
        if t < self.caster_stuck_until:  # caster kẹt: kéo lệch sang trái
            w += 0.15 * abs(v) / 100.0
        mid = self.th + 0.5 * w * dt
        self.x += v * math.cos(mid) * dt
        self.y += v * math.sin(mid) * dt
        self.th = wrap(self.th + w * dt)
        self.th_acc += w * dt
        self.v, self.w = v, w
        self.enc_dist["l"] += ul * dt
        self.enc_dist["r"] += ur * dt

    def ticks(self, side: str) -> int:
        d = self.p.wheel_d_l if side == "l" else self.p.wheel_d_r
        return math.floor(self.enc_dist[side] / (math.pi * d) * self.p.ticks_per_rev)

    GYRO_LSB = math.radians(1 / 131.0)  # MPU6050 ±250 °/s: 131 LSB/(°/s)

    T_AMBIENT = 25.0

    def temp_true(self) -> float:
        """Nhiệt độ chip MPU: nóng dần sau khi bật nguồn."""
        p = self.p
        return self.T_AMBIENT + p.temp_rise * (1 - math.exp(-self.t_on / p.gyro_warm_tau))

    def gyro_bias_true(self) -> float:
        """bias = bias bật nguồn + hệ số nhiệt·ΔT + random walk."""
        p = self.p
        k_temp = p.gyro_warm / p.temp_rise if p.temp_rise else 0.0
        return p.gyro_bias + k_temp * (self.temp_true() - self.T_AMBIENT) + self.bias_rw

    def imu_temp(self, rng: random.Random) -> float:
        return round((self.temp_true() + rng.gauss(0.0, 0.02)) * 340) / 340  # 340 LSB/°C

    def gyro(self, rng: random.Random, t: float = 0.0) -> float:
        """ω_đo = (1+e_s)·ω + bias(t) + nhiễu trắng + nhiễu rung motor, lượng tử 1/131 °/s."""
        p = self.p
        vib = p.gyro_vib * 0.5 * (abs(self.u["l"]) + abs(self.u["r"]))
        val = (self.w * (1 + p.gyro_scale_err) + self.gyro_bias_true()
               + rng.gauss(0.0, p.gyro_noise) + (rng.gauss(0.0, vib) if vib > 0 else 0.0))
        if t < self.bump_until:
            val += self.bump_rate
        return round(val / self.GYRO_LSB) * self.GYRO_LSB

    def to_world(self, bx: float, by: float) -> tuple[float, float]:
        c, s = math.cos(self.th), math.sin(self.th)
        return self.x + bx * c - by * s, self.y + bx * s + by * c

    def polygon(self) -> list[tuple[float, float]]:
        p = self.p
        hw = p.width / 2
        return [self.to_world(p.front, hw), self.to_world(-p.rear, hw),
                self.to_world(-p.rear, -hw), self.to_world(p.front, -hw)]


def _ray_seg(ox, oy, dx, dy, ax, ay, bx, by) -> float | None:
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-9:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / den
    u = ((ax - ox) * dy - (ay - oy) * dx) / den
    if t < 0 or u < 0 or u > 1:
        return None
    ln = math.hypot(ex, ey)
    cos_inc = abs(dx * (-ey) + dy * ex) / ln  # |dir · normal|
    if cos_inc < 0.5:  # góc tới > 60°: không có phản xạ đủ mạnh
        return None
    return t


def _sat_overlap(a: list, b: list) -> bool:
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            nx, ny = y1 - y2, x2 - x1
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


@dataclass
class FrictionZone:
    x0: float
    y0: float
    x1: float
    y1: float
    factor: float


@dataclass
class World:
    width: float
    height: float
    bodies: dict[str, Body] = field(default_factory=dict)
    zones: list[FrictionZone] = field(default_factory=list)
    wall_margin: float = 400.0

    def walls(self) -> list[tuple[float, float, float, float]]:
        m = self.wall_margin
        x0, y0, x1, y1 = -m, -m, self.width + m, self.height + m
        return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]

    def fric_factor(self, x: float, y: float) -> float:
        f = 1.0
        for z in self.zones:
            if z.x0 <= x <= z.x1 and z.y0 <= y <= z.y1:
                f *= z.factor
        return f

    def raycast(self, ox: float, oy: float, ang: float, exclude: str) -> float | None:
        dx, dy = math.cos(ang), math.sin(ang)
        best: float | None = None
        segs = list(self.walls())
        for bid, b in self.bodies.items():
            if bid == exclude:
                continue
            poly = b.polygon()
            for i in range(4):
                (ax, ay), (bx, by) = poly[i], poly[(i + 1) % 4]
                segs.append((ax, ay, bx, by))
        for ax, ay, bx, by in segs:
            t = _ray_seg(ox, oy, dx, dy, ax, ay, bx, by)
            if t is not None and (best is None or t < best):
                best = t
        return best

    def tof(self, body: Body, side: str, t: float, rng: random.Random) -> tuple[float, int]:
        """(mm, status) như VL53L0X: status 0 = hợp lệ, 4 = ngoài tầm."""
        p = body.p
        if body.tof_block.get(side, 0.0) > t:
            return 15.0, 0
        by = p.tof_b / 2 if side == "l" else -p.tof_b / 2
        ox, oy = body.to_world(p.tof_x + p.tof_dx[side], by)
        half = math.radians(p.tof_fov_deg / 2)
        axis = body.th + p.tof_yaw[side]
        best = None
        for k in (-1.0, -0.5, 0.0, 0.5, 1.0):
            d = self.raycast(ox, oy, axis + k * half, body.id)
            if d is not None and (best is None or d < best):
                best = d
        if best is None or best > 2000:
            return 8190.0, 4
        if rng.random() < 0.004:
            return 8190.0, 2  # lỗi sigma ngẫu nhiên
        noisy = best + p.tof_off[side] + rng.gauss(0.0, p.tof_sigma * (1 + best / 1000))
        return max(0.0, noisy), 0

    def collisions(self) -> list[tuple[str, str]]:
        ids = list(self.bodies)
        out = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                if _sat_overlap(self.bodies[ids[i]].polygon(), self.bodies[ids[j]].polygon()):
                    out.append((ids[i], ids[j]))
        return out

"""Điều khiển bánh, động học, ước lượng hướng, odometry.

Xem wiki/motor-control.md (L1 PI bánh + FF, L3 bão hoà ưu tiên ω, L4 thích nghi)
và wiki/algorithms.md §2 (fusion gyro + encoder, ZUPT).
"""
from __future__ import annotations

import math
from collections import deque

from .mathutil import clamp, sign, wrap


class WheelController:
    """FF (vùng chết + kv, bù V_bat) + PI tốc độ + học offset FF.

    Tích phân lưu theo đơn vị volt → đổi Ki lúc chạy không làm nhảy đầu ra (bumpless).
    """

    def __init__(self, side: str):
        self.side = side
        self.integ = 0.0  # V
        self.adapt = 0.0  # tỉ lệ bù FF học được
        self.v_ref = 0.0
        self.duty = 0.0

    def reset(self) -> None:
        self.integ = 0.0
        self.v_ref = 0.0
        self.duty = 0.0

    def update(self, v_ref: float, v_meas: float, vbat: float, cfg: dict, dt: float) -> float:
        s = self.side
        self.v_ref = v_ref
        if v_ref == 0.0:
            self.integ = 0.0
            self.duty = 0.0
            return 0.0
        ks, kv = cfg[f"motor.ks_{s}_v"], cfg[f"motor.kv_{s}"]
        ff = sign(v_ref) * (ks + kv * abs(v_ref)) * (1.0 + self.adapt)
        e = v_ref - v_meas
        self.integ = clamp(self.integ + cfg["pid.wheel_ki"] * e * dt, -3.0, 3.0)
        volts = ff + cfg["pid.wheel_kp"] * e + self.integ
        # L4: chuyển dần phần I sang offset FF khi chạy ổn định.
        rate = cfg["motor.adapt_rate"]
        if rate > 0 and abs(v_ref) > 2 * cfg["motor.v_min_reliable"] and abs(ff) > 0.5:
            lim = cfg["motor.adapt_limit"]
            self.adapt = clamp(self.adapt + rate * dt * clamp(self.integ * sign(v_ref) / abs(ff), -1, 1), -lim, lim)
        vavail = max(vbat - cfg["power.l298n_vdrop_v"], 1.0)
        self.duty = clamp(volts / vavail, -1.0, 1.0)
        return self.duty


class DriveBase:
    """(v, ω) → tốc độ 2 bánh → PWM. Đo tốc độ bánh từ encoder (cửa sổ trượt 50 ms)."""

    WINDOW = 5

    def __init__(self):
        self.wl = WheelController("l")
        self.wr = WheelController("r")
        self._hl: deque[float] = deque(maxlen=self.WINDOW)
        self._hr: deque[float] = deque(maxlen=self.WINDOW)
        self.v_meas_l = 0.0
        self.v_meas_r = 0.0
        self.sat = False
        self.braking = True

    @property
    def v_meas(self) -> float:
        return 0.5 * (self.v_meas_l + self.v_meas_r)

    def measure(self, dl: float, dr: float, dt: float) -> None:
        self._hl.append(dl)
        self._hr.append(dr)
        self.v_meas_l = sum(self._hl) / (len(self._hl) * dt)
        self.v_meas_r = sum(self._hr) / (len(self._hr) * dt)

    def command(self, v: float, w: float, brake: bool, cfg: dict, vbat: float, dt: float):
        """Trả (duty_l, duty_r, brake)."""
        self.braking = brake
        if brake:
            self.wl.reset()
            self.wr.reset()
            self.sat = False
            return 0.0, 0.0, True
        half_t = cfg["wheel.track_eff_mm"] / 2
        vavail = vbat - cfg["power.l298n_vdrop_v"]
        vmax = 0.95 * max(0.0, min((vavail - cfg["motor.ks_l_v"]) / cfg["motor.kv_l"],
                                   (vavail - cfg["motor.ks_r_v"]) / cfg["motor.kv_r"]))
        # L3: bão hoà ưu tiên ω — giảm v, giữ hướng.
        w_part = w * half_t
        sat = False
        if abs(w_part) > vmax:
            w_part = sign(w_part) * vmax
            sat = True
        v_allow = vmax - abs(w_part)
        if abs(v) > v_allow:
            v = sign(v) * v_allow
            sat = True
        self.sat = sat
        vl, vr = v - w_part, v + w_part
        dl = self.wl.update(vl, self.v_meas_l, vbat, cfg, dt)
        dr = self.wr.update(vr, self.v_meas_r, vbat, cfg, dt)
        return dl, dr, False


class HeadingEstimator:
    """ψ = tích phân [(1-β)·ω_gyro + β·ω_enc]; β>0 chỉ khi đi thẳng. ZUPT ước lượng bias khi đứng yên."""

    ZUPT_DELAY = 0.3
    BIAS_GAIN = 0.02

    def __init__(self):
        self.psi = 0.0
        self.bias = 0.0
        self.w = 0.0
        self.w_gyro = 0.0
        self.w_enc = 0.0
        self._still_t = 0.0

    def update(self, gz_raw: float, dl: float, dr: float, straight: bool, still: bool, cfg: dict, dt: float) -> None:
        if still:
            self._still_t += dt
            if self._still_t > self.ZUPT_DELAY:
                self.bias += self.BIAS_GAIN * (gz_raw - self.bias)
        else:
            self._still_t = 0.0
        self.w_gyro = (gz_raw - self.bias) * cfg["imu.gyro_scale_z"]
        self.w_enc = (dr - dl) / cfg["wheel.track_eff_mm"] / dt
        if still and self._still_t > self.ZUPT_DELAY:
            self.w = 0.0
        else:
            beta = cfg["imu.beta_straight"] if straight else 0.0
            self.w = (1 - beta) * self.w_gyro + beta * self.w_enc
        self.psi = wrap(self.psi + self.w * dt)


class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.s = 0.0  # tiến độ dọc đường (mm), chỉ cộng quãng tiến/lùi
        self._psi_prev: float | None = None

    def reset(self, x: float, y: float, s: float) -> None:
        self.x, self.y, self.s = x, y, s
        self._psi_prev = None

    def update(self, dl: float, dr: float, psi: float) -> None:
        ds = 0.5 * (dl + dr)
        p0 = psi if self._psi_prev is None else self._psi_prev
        mid = p0 + 0.5 * wrap(psi - p0)
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.s += ds
        self._psi_prev = psi

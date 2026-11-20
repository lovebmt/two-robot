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


class ImuCalibrator:
    """Calib bias gyro khi xe đứng yên (wiki/algorithms.md §2.1).

    Lấy trung bình gyro trong imu.calib_time_s. Nếu encoder nhích hoặc gyro dao động quá
    imu.calib_max_spread_dps (bị chạm, rung) thì làm lại từ đầu; quá MAX_RESTART lần → lỗi.
    """

    MAX_RESTART = 5

    def __init__(self, cfg: dict):
        self.dur = cfg["imu.calib_time_s"]
        self.max_spread = math.radians(cfg["imu.calib_max_spread_dps"])
        self.restarts = 0
        self.moved_restarts = 0
        self._reset()

    def _reset(self) -> None:
        self.n, self.s, self.s2, self.t = 0, 0.0, 0.0, 0.0
        self.lo, self.hi = math.inf, -math.inf

    @property
    def progress(self) -> float:
        return min(1.0, self.t / self.dur)

    def feed(self, gz_raw: float, moved: bool, dt: float) -> dict | None:
        """Trả None khi chưa xong; dict {ok, bias, std, spread, ...} khi xong hoặc thất bại."""
        if moved or (self.n and max(self.hi, gz_raw) - min(self.lo, gz_raw) > self.max_spread):
            self.restarts += 1
            self.moved_restarts += 1 if moved else 0
            self._reset()
            if self.restarts > self.MAX_RESTART:
                # Bánh không hề nhích mà gyro vẫn dao động quá ngưỡng → gyro nhiễu (hỏng, rung từ bên ngoài),
                # không phải bị chạm. Có thể nới imu.calib_max_spread_dps nếu chấp nhận gyro nhiễu.
                err = "IMU_NOT_STILL" if self.moved_restarts else "IMU_NOISY"
                return {"ok": False, "err": err, "restarts": self.restarts}
            return None
        self.n += 1
        self.s += gz_raw
        self.s2 += gz_raw * gz_raw
        self.lo, self.hi = min(self.lo, gz_raw), max(self.hi, gz_raw)
        self.t += dt
        if self.t < self.dur:
            return None
        mean = self.s / self.n
        std = math.sqrt(max(0.0, self.s2 / self.n - mean * mean))
        return {"ok": True, "bias": mean, "std": std, "spread": self.hi - self.lo,
                "n": self.n, "restarts": self.restarts}


class HeadingEstimator:
    """ψ = tích phân [(1-β)·ω_gyro + β·ω_enc]; β>0 chỉ khi đi thẳng (wiki/algorithms.md §2).

    Bias gyro:
      1. ImuCalibrator đặt bias lúc đầu (xe đứng yên).
      2. ZUPT: mỗi khi xe đứng yên (bánh phanh, encoder không nhích, |ω gyro| nhỏ) lọc bias
         theo số đo → bám bias trôi (nóng máy, random walk).
      3. Bù nhiệt: bias trôi chủ yếu theo nhiệt độ chip. Mỗi lần đứng yên đủ lâu ghi một điểm
         (T, bias); hồi quy tuyến tính → hệ số k (rad/s/°C). Khi chạy: bias = bias_neo + k·(T − T_neo).
    Không học bias bằng encoder khi đang chạy: lệch đường kính 2 bánh 1 % đã gây ~0.7 °/s sai ω_enc
    ở 150 mm/s, lớn hơn chính bias cần bám.
    """

    ZUPT_DELAY = 0.3
    ANCHOR_MIN_STILL = 1.0        # s: chỉ lấy điểm (T, bias) từ lần đứng yên đủ lâu
    ANCHOR_PERIOD = 5.0           # s: khi đứng yên lâu, ghi điểm định kỳ
    MAX_ANCHORS = 24
    MIN_TEMP_SPAN = 0.5           # °C: cần chênh nhiệt tối thiểu để hồi quy
    K_MAX = math.radians(0.5)     # rad/s/°C

    def __init__(self):
        self.psi = 0.0
        self.bias = 0.0
        self.k_temp: float | None = None   # None → dùng giá trị trong config
        self.temp = 25.0
        self.w = 0.0
        self.w_gyro = 0.0
        self.w_enc = 0.0
        self.zupt_active = False
        self._still_t = 0.0
        self._anchor_t = 0.0
        self._anchors: list[tuple[float, float]] = []
        self._ref: tuple[float, float] | None = None  # (T, bias) neo gần nhất

    def k(self, cfg: dict) -> float:
        return self.k_temp if self.k_temp is not None else math.radians(cfg["imu.temp_coeff_dps_c"])

    def anchor_bias(self) -> None:
        """Ghi điểm (T, bias) tin cậy (sau calib hoặc khi kết thúc lần ZUPT đủ dài), học lại k."""
        self._ref = (self.temp, self.bias)
        self._anchors.append(self._ref)
        if len(self._anchors) > self.MAX_ANCHORS:
            self._anchors.pop(0)
        ts = [a[0] for a in self._anchors]
        if max(ts) - min(ts) < self.MIN_TEMP_SPAN:
            return
        n = len(ts)
        mt = sum(ts) / n
        mb = sum(a[1] for a in self._anchors) / n
        sxx = sum((t - mt) ** 2 for t in ts)
        sxy = sum((t - mt) * (b - mb) for t, b in self._anchors)
        self.k_temp = max(-self.K_MAX, min(self.K_MAX, sxy / sxx))

    def update(self, gz_raw: float, dl: float, dr: float, straight: bool, still: bool, cfg: dict, dt: float,
               calibrating: bool = False, temp: float | None = None) -> None:
        if temp is not None:
            self.temp = temp
        still = still and abs(gz_raw - self.bias) < math.radians(cfg["imu.still_gyro_dps"])
        if still:
            self._still_t += dt
        else:
            if self.zupt_active and self._still_t > self.ANCHOR_MIN_STILL and cfg["imu.zupt_enable"]:
                self.anchor_bias()
            self._still_t = 0.0
            self._anchor_t = 0.0
        self.zupt_active = still and self._still_t > self.ZUPT_DELAY
        if self.zupt_active and cfg["imu.zupt_enable"] and not calibrating:
            self.bias += cfg["imu.zupt_gain"] * (gz_raw - self.bias)
            # Đứng yên lâu (vd. vừa bật nguồn, chip đang nóng dần): ghi điểm định kỳ để học k sớm.
            self._anchor_t += dt
            if self._still_t > self.ANCHOR_MIN_STILL and self._anchor_t >= self.ANCHOR_PERIOD:
                self._anchor_t = 0.0
                self.anchor_bias()
        elif not calibrating and cfg["imu.temp_comp"] and self._ref is not None:
            # Đang chạy: bias theo nhiệt độ hiện tại so với điểm neo gần nhất.
            self.bias = self._ref[1] + self.k(cfg) * (self.temp - self._ref[0])
        self.w_gyro = (gz_raw - self.bias) * cfg["imu.gyro_scale_z"]
        self.w_enc = (dr - dl) / cfg["wheel.track_eff_mm"] / dt
        if (self.zupt_active and cfg["imu.zupt_enable"]) or calibrating:
            self.w = 0.0  # đứng yên: không tích phân góc
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

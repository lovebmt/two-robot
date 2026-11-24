"""Primitive chuyển động — PC chỉ ra lệnh ở mức này (wiki/architecture.md §3.1).

Mỗi primitive: start(fw) → update(fw, dt) trả (v, ω, brake) mỗi chu kỳ 100 Hz,
on_range(fw, meas) khi có mẫu ToF mới. Kết thúc bằng finish() → firmware gửi evt prim_done.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

from .mathutil import clamp, sign, wrap
from .motion_profile import JerkRamp
from .rangeproc import RangeMeas

if TYPE_CHECKING:
    from .robot import VirtualFirmware

BRAKE = (0.0, 0.0, True)


class RAW(tuple):
    """Lệnh điện áp thẳng xuống 2 motor (CAL_KS), bỏ qua PI bánh. Vẫn là bộ (v, ω, brake) để giải nén."""

    def __new__(cls, volt_l: float, volt_r: float):
        t = super().__new__(cls, (0.0, 0.0, False))
        t.volts = (volt_l, volt_r)
        return t


class Prim:
    name = "?"
    standalone = False  # True: chạy được cả khi xe IDLE (hiệu chuẩn, kiểm tra), không cần path/ARM
    needs_imu = True    # khi chạy từ IDLE: cần gyro đã calib

    def __init__(self, pid: int, args: dict):
        self.id = pid
        self.args = args
        self.done = False
        self.ok = True
        self.err: str | None = None
        self.result: dict = {}

    def start(self, fw: "VirtualFirmware") -> None:
        pass

    def update(self, fw: "VirtualFirmware", dt: float):
        return BRAKE

    def on_range(self, fw: "VirtualFirmware", m: RangeMeas) -> None:
        pass

    def finish(self, ok: bool = True, err: str | None = None, **result) -> None:
        self.done, self.ok, self.err = True, ok, err
        self.result = result


def _heading_hold(fw: "VirtualFirmware", target: float, integ: list[float], dt: float) -> float:
    cfg = fw.params.active
    e = wrap(target - fw.heading.psi)
    integ[0] = clamp(integ[0] + cfg["pid.heading_ki"] * e * dt, -0.3, 0.3)
    return cfg["pid.heading_kp"] * e + integ[0]


class DriveTo(Prim):
    """Đi thẳng tới tiến độ s_end, giữ hướng, profile hình thang, chậm dần và dừng tại điểm."""
    name = "DRIVE_TO"

    def start(self, fw):
        cfg = fw.params.active
        self.s_end = float(self.args["s_end"])
        self.v_max = float(self.args.get("v_max") or cfg["motion.v_cruise"])
        h = self.args.get("heading_deg")
        self.hold = math.radians(h) if h is not None else fw.heading.psi
        self.hint = [0.0]
        self.ramp = JerkRamp()
        self.ramp.reset(max(0.0, fw.drive.v_meas))
        self.stall = 0.0

    def update(self, fw, dt):
        cfg = fw.params.active
        rem = self.s_end - fw.odo.s
        # Kẹt sát đích (ma sát tĩnh) quá stall_time → coi như tới, không chờ mãi
        near = rem <= cfg["motion.settle_tol_mm"] and abs(fw.drive.v_meas) < 2.0
        self.stall = self.stall + dt if near else 0.0
        if rem <= cfg["motion.stop_tol_mm"] or self.stall > cfg["motion.stall_time_s"]:
            extra = {"stalled": True} if rem > cfg["motion.stop_tol_mm"] else {}
            self.finish(s=fw.odo.s, yaw_deg=math.degrees(fw.heading.psi), overshoot_mm=-rem, **extra)
            return BRAKE
        a, vc, app = cfg["motion.a_max"], cfg["motion.v_corner"], cfg["motion.approach_dist"]
        if rem > app:
            v_target = min(self.v_max, math.sqrt(vc * vc + 2 * a * (rem - app)))
        else:
            v_target = max(min(vc, cfg["pid.pos_kp"] * rem), cfg["motor.v_min_reliable"])
        v = clamp(self.ramp.step(v_target, a, dt), 0.0, self.v_max)
        return v, _heading_hold(fw, self.hold, self.hint, dt), False


def turn_w_min(cfg: dict) -> float:
    """ω tối thiểu khi quay tại chỗ: bánh không chậm hơn motor.v_min_reliable (dưới mức đó motor không kéo nổi)."""
    return max(math.radians(6.0), cfg["motor.v_min_reliable"] / max(cfg["wheel.track_eff_mm"] / 2.0, 1.0))


class Turn(Prim):
    """Quay tại chỗ vòng kín theo hướng (gyro). args: deg (tương đối) hoặc heading_deg (tuyệt đối)."""
    name = "TURN"

    def start(self, fw):
        if self.args.get("heading_deg") is not None:
            tgt = math.radians(float(self.args["heading_deg"]))
        else:
            tgt = fw.heading.psi + math.radians(float(self.args.get("deg", 0.0)))
        self.target = wrap(tgt)
        self.ramp = JerkRamp()
        self.settle = 0.0
        self.stall = 0.0

    def update(self, fw, dt):
        cfg = fw.params.active
        e = wrap(self.target - fw.heading.psi)
        tol = math.radians(cfg["motion.turn_tol_deg"])
        near = abs(e) < math.radians(cfg["motion.settle_tol_deg"]) and abs(fw.heading.w) < math.radians(2.0)
        self.stall = self.stall + dt if near else 0.0
        stuck = self.stall > cfg["motion.stall_time_s"]
        if abs(e) < tol or stuck:
            self.settle += dt
            if self.settle > 0.15 or stuck:
                extra = {"stalled": True} if abs(e) >= tol else {}
                self.finish(yaw_deg=math.degrees(fw.heading.psi), err_deg=math.degrees(e), **extra)
            return BRAKE
        self.settle = 0.0
        wmax = math.radians(cfg["motion.turn_rate_dps"])
        alpha = math.radians(cfg["motion.turn_acc_dps2"])
        w_target = min(wmax, 0.4 * math.sqrt(2 * alpha * abs(e)))
        w_target = max(w_target, turn_w_min(cfg))
        w = self.ramp.step(sign(e) * w_target, alpha, dt)
        return 0.0, w, False


class Hold(Prim):
    name = "HOLD"

    def start(self, fw):
        self.finish()


class Follow(Prim):
    """Bám leader trên đoạn thẳng, tự chủ trên xe (wiki/algorithms.md §4)."""
    name = "FOLLOW"

    def start(self, fw):
        cfg = fw.params.active
        a = self.args
        self.g_t = float(a.get("g_target") or cfg["gap.target_mm"])
        self.g_min = float(a.get("gap_min_safe") or 0.0)
        self.s_limit = a.get("s_limit")
        self.v_max = float(a.get("v_max") or cfg["motion.v_cruise"] * cfg["follow.v_headroom"])
        m = fw.meas
        now = fw.hal.now_ms()
        fresh = m is not None and m.valid and now - m.t_ms < 200
        fw.kf.reset(m.rho if fresh else self.g_t, 0.0, cfg["range.sigma_mm"])
        self.integ = 0.0
        self.ramp = JerkRamp()
        self.ramp.reset(max(0.0, fw.drive.v_meas))
        self.dpsi_f = 0.0
        self.have_dpsi = False
        self.edge = 0
        self.hold = fw.heading.psi
        self.hint = [0.0]
        self.last_valid_ms = now
        self.last_dpsi_ms = -10_000

    def on_range(self, fw, m):
        if not m.valid:
            return
        cfg = fw.params.active
        kf = fw.kf
        ok = kf.update_g(m.rho, cfg["range.sigma_mm"], cfg["follow.kf_gate_chi2"])
        if not ok and kf.rejects >= 8:  # KF lệch hẳn → khởi tạo lại theo số đo
            kf.reset(m.rho, kf.vL, cfg["range.sigma_mm"])
            ok = True
        if not ok:
            return
        self.last_valid_ms = m.t_ms
        if m.dpsi is not None:
            self.dpsi_f = m.dpsi if not self.have_dpsi else self.dpsi_f + 0.4 * (m.dpsi - self.dpsi_f)
            self.have_dpsi = True
            self.last_dpsi_ms = m.t_ms
            self.edge = 0
        else:
            self.edge = m.edge

    def update(self, fw, dt):
        cfg = fw.params.active
        now = fw.hal.now_ms()
        if now - self.last_valid_ms > cfg["follow.lost_timeout_ms"]:
            fw.raise_fault("LEADER_LOST")
            return BRAKE
        kf = fw.kf
        kf.predict(fw.drive.v_meas, dt, cfg["follow.kf_q_a"])
        if fw.relay is not None and now - fw.relay[0] < 300:
            kf.update_v(fw.relay[1], 40.0)
        e = kf.g - self.g_t
        self.integ = clamp(self.integ + cfg["follow.ki_g"] * e * dt, -60.0, 60.0)
        v_target = kf.vL + cfg["follow.kp_g"] * e + self.integ
        a = cfg["motion.a_max"]
        v_target = clamp(v_target, 0.0, self.v_max)
        if self.s_limit is not None:
            rem = float(self.s_limit) - fw.odo.s
            if rem <= cfg["motion.stop_tol_mm"]:
                self.finish(s=fw.odo.s, gap=kf.g)
                return BRAKE
            v_target = min(v_target, math.sqrt(2 * a * rem))
        if kf.g < self.g_min:
            self.integ = min(self.integ, 0.0)
            self.ramp.reset()
            return BRAKE
        v = self.ramp.step(v_target, a, dt)
        # Hướng: song song với leader theo Δψ (ToF); mất Δψ thì giữ hướng bằng gyro.
        self.dpsi_f -= fw.heading.w * dt
        if self.have_dpsi and now - self.last_dpsi_ms < 250:
            w = cfg["follow.k_psi"] * self.dpsi_f
            self.hold = fw.heading.psi
        else:
            w = _heading_hold(fw, self.hold, self.hint, dt) + cfg["follow.k_edge"] * self.edge
        if v < 1.0:
            return BRAKE
        return v, w, False


class FollowPlate(Prim):
    """Chế độ 1 xe bám tấm phẳng cầm tay (không cần path/ARM). Giữ mặt kính ToF → tấm = plate.dist_mm: tấm xa thì
    tiến, gần thì lùi (≤ plate.v_back), căn song song theo Δψ. Mất tấm → phanh và CHỜ (không lỗi); thấy lại → bám
    tiếp. Chạy tới khi PC gửi stop. Tham số nhóm "plate" chép vào biến 1 lần lúc start, dùng suốt lượt chạy
    (nhóm "plate" là apply=idle: không cho sửa khi xe đang chạy); args ghi đè cho riêng lượt này."""
    name = "FOLLOW_PLATE"
    standalone = True

    def _arg(self, k, v):
        return float(self.args[k]) if self.args.get(k) is not None else v

    def start(self, fw):
        cfg = fw.params.active
        self.g_t = cfg["range.x_mm"] + self._arg("d_target", cfg["plate.dist_mm"]) + fw.range_rear()
        self.band = self._arg("hold_band", cfg["plate.hold_band_mm"])
        self.v_max = self._arg("v_max", cfg["plate.v_max"])
        self.v_back = self._arg("v_back", cfg["plate.v_back"])
        self.kp, self.k_psi, self.k_edge = cfg["plate.kp"], cfg["plate.k_psi"], cfg["plate.k_edge"]
        self.max_dpsi = math.radians(cfg["plate.max_dpsi_deg"])
        self.integ = self.v_prev = self.dpsi_f = 0.0
        self.have_dpsi, self.lost, self.edge = False, True, 0
        self.hold = fw.heading.psi
        self.hint = [0.0]
        self.last_valid_ms = 0
        self.last_dpsi_ms = -10_000

    def on_range(self, fw, m):
        if not m.valid:
            return
        cfg = fw.params.active
        kf = fw.kf
        # Tấm cầm tay xoay nhiều: tự tính Δψ khi 2 mắt cùng thấy, cho phép tới plate.max_dpsi_deg
        # (rangeproc chỉ nhận |dl−dr| ≤ range.max_diff ≈ 18° — dành cho đuôi xe trước)
        rho, dps, pair = m.rho, 0.0, False
        if m.dl is not None and m.dr is not None:
            dps = math.atan2(m.dr - m.dl, cfg["range.baseline_mm"])
            if abs(dps) < self.max_dpsi:
                pair = True
                rho = cfg["range.x_mm"] + 0.5 * (m.dl + m.dr) * math.cos(dps) + fw.range_rear()
        if self.lost:  # (bắt) lại tấm: khởi tạo KF theo số đo
            kf.reset(rho, 0.0, cfg["range.sigma_mm"])
            self.integ = self.v_prev = 0.0
            self.have_dpsi, self.lost = False, False
        elif not kf.update_g(rho, cfg["range.sigma_mm"], cfg["follow.kf_gate_chi2"]):
            if kf.rejects < 8:
                return
            kf.reset(rho, kf.vL, cfg["range.sigma_mm"])
        self.last_valid_ms = m.t_ms
        if pair:
            self.dpsi_f = dps if not self.have_dpsi else self.dpsi_f + 0.5 * (dps - self.dpsi_f)
            self.have_dpsi, self.last_dpsi_ms, self.edge = True, m.t_ms, 0
        else:
            self.edge = m.edge

    def update(self, fw, dt):
        cfg = fw.params.active
        now = fw.hal.now_ms()
        if self.lost or now - self.last_valid_ms > cfg["follow.lost_timeout_ms"]:
            self.lost = True
            self.hold = fw.heading.psi
            return BRAKE
        band, v_max, v_back = self.band, self.v_max, self.v_back
        kf = fw.kf
        kf.predict(fw.drive.v_meas, dt, cfg["follow.kf_q_a"])
        e = kf.g - self.g_t  # > 0: tấm xa hơn đích → tiến
        self.integ = (clamp(self.integ + cfg["follow.ki_g"] * e * dt, -40.0, 40.0)
                      if abs(e) < 4 * band + 20.0 else 0.0)
        v = kf.vL + self.kp * e + self.integ
        if abs(e) < band and abs(kf.vL) < 15.0:
            v, self.integ = 0.0, 0.0
        v = clamp(v, -v_back, v_max)
        a = cfg["motion.a_max"]
        v = clamp(v, self.v_prev - a * dt, self.v_prev + a * dt)
        self.v_prev = v
        self.dpsi_f -= fw.heading.w * dt
        if self.have_dpsi and now - self.last_dpsi_ms < 250:
            w = self.k_psi * self.dpsi_f
            self.hold = fw.heading.psi
        elif self.edge:  # 1 mắt mất tấm: xoay hẳn về phía mắt còn thấy
            w = self.k_edge * self.edge
            self.hold = fw.heading.psi
        else:
            w = _heading_hold(fw, self.hold, self.hint, dt)
        wmax = math.radians(cfg["motion.turn_rate_dps"])
        w = clamp(w, -wmax, wmax)
        if abs(v) < 1.0 and abs(w) < 0.05:
            return BRAKE
        return v, w, False


class Snapshot(Prim):
    """Đứng yên, lấy trung bình N mẫu ToF → ρ, Δψ (mốc neo ở góc cua; kiểm tra trước khi chạy;
    calib khoảng cách). Trả cả số đo trung bình từng mắt (đã bù offset) và offset đang dùng."""
    name = "SNAPSHOT"
    standalone = True
    needs_imu = False

    def start(self, fw):
        cfg = fw.params.active
        self.n = int(self.args.get("n") or cfg["corner.snapshot_n"])
        self.samples: list[tuple[float, float, float, float]] = []
        self.t0 = fw.hal.now_ms()
        self.seen_l = self.seen_r = 0

    def on_range(self, fw, m):
        self.seen_l += m.dl is not None
        self.seen_r += m.dr is not None
        if m.valid and m.dpsi is not None:
            self.samples.append((m.rho, m.dpsi, m.dl, m.dr))

    def update(self, fw, dt):
        if len(self.samples) >= self.n:
            k = len(self.samples)
            rhos = [x[0] for x in self.samples]
            mean = sum(rhos) / k
            std = math.sqrt(sum((r - mean) ** 2 for r in rhos) / k)
            cfg = fw.params.active
            self.finish(rho=mean, std=std, dpsi_deg=math.degrees(sum(x[1] for x in self.samples) / k),
                        dl_mm=sum(x[2] for x in self.samples) / k, dr_mm=sum(x[3] for x in self.samples) / k,
                        offset_l_mm=cfg["range.offset_l_mm"], offset_r_mm=cfg["range.offset_r_mm"],
                        s=fw.odo.s, n=k)
        elif fw.hal.now_ms() - self.t0 > 3000:
            # Phân biệt: không thấy gì / chỉ 1 mắt thấy (lệch ngang, mép tấm) / 2 mắt chênh quá lớn
            err = ("NO_TARGET" if not (self.seen_l or self.seen_r)
                   else "ONE_EYE_ONLY" if not (self.seen_l and self.seen_r) else "EYES_DISAGREE")
            self.finish(ok=False, err=err, n=len(self.samples), seen_l=self.seen_l, seen_r=self.seen_r)
        return BRAKE


class _Sampler:
    def __init__(self, need: int, need_dpsi: bool):
        self.need, self.need_dpsi = need, need_dpsi
        self.rho: list[float] = []
        self.dpsi: list[float] = []

    def add(self, m: RangeMeas) -> None:
        if not m.valid or (self.need_dpsi and m.dpsi is None):
            return
        self.rho.append(m.rho)
        if m.dpsi is not None:
            self.dpsi.append(m.dpsi)

    @property
    def full(self) -> bool:
        return len(self.rho) >= self.need

    def mean_rho(self) -> float:
        return sum(self.rho) / len(self.rho)

    def mean_dpsi(self) -> float:
        return sum(self.dpsi) / len(self.dpsi)


class Align(Prim):
    """Quay tại chỗ tới khi song song với leader (Δψ ≈ 0) theo ToF."""
    name = "ALIGN"
    MAX_ITER = 4

    def start(self, fw):
        cfg = fw.params.active
        self.tol = math.radians(float(self.args.get("tol_deg") or cfg["corner.align_tol_deg"]))
        self.iter = 0
        self._measure(fw)

    def _measure(self, fw):
        # Δψ từ 2 ToF rất nhiễu khi baseline nhỏ (σ_Δψ ≈ √2·σ_d / b) → lấy trung bình nhiều mẫu.
        self.phase = "measure"
        self.sampler = _Sampler(int(fw.params.active["corner.snapshot_n"]), True)
        self.t0 = fw.hal.now_ms()
        self.turn: Turn | None = None

    def on_range(self, fw, m):
        if self.phase == "measure":
            self.sampler.add(m)

    def update(self, fw, dt):
        if self.phase == "measure":
            if self.sampler.full:
                d = self.sampler.mean_dpsi()
                if abs(d) < self.tol or self.iter >= self.MAX_ITER:
                    self.finish(dpsi_deg=math.degrees(d), iters=self.iter)
                    return BRAKE
                self.iter += 1
                self.turn = Turn(-1, {"deg": math.degrees(d)})
                self.turn.start(fw)
                self.phase = "turn"
            elif fw.hal.now_ms() - self.t0 > 2000:
                self.finish(ok=False, err="NO_TARGET")
            return BRAKE
        out = self.turn.update(fw, dt)
        if self.turn.done:
            self._measure(fw)
        return out


class Reacquire(Prim):
    """Sau khi quay: xác nhận leader ở trước mặt ở khoảng ρ_exp; không thấy thì quét ±scan."""
    name = "REACQUIRE"

    def start(self, fw):
        cfg = fw.params.active
        self.rho_exp = float(self.args["rho_exp"])
        self.tol = float(self.args.get("tol") or cfg["corner.reacquire_tol_mm"])
        scan = float(self.args.get("scan_deg") if self.args.get("scan_deg") is not None else cfg["corner.scan_deg"])
        self.base = fw.heading.psi
        self.offsets = [0.0]
        k = 3.0
        while k <= scan + 1e-6:
            self.offsets += [-k, k]
            k += 3.0
        self.idx = 0
        self._begin_measure(fw)

    def _begin_measure(self, fw):
        self.phase = "measure"
        self.sampler = _Sampler(4, False)
        self.t0 = fw.hal.now_ms()

    def _next(self, fw):
        self.idx += 1
        if self.idx >= len(self.offsets):
            self.finish(ok=False, err="LEADER_NOT_FOUND")
            return
        tgt = math.degrees(self.base) + self.offsets[self.idx]
        self.turn = Turn(-1, {"heading_deg": tgt})
        self.turn.start(fw)
        self.phase = "turn"

    def on_range(self, fw, m):
        if self.phase == "measure":
            self.sampler.add(m)

    def update(self, fw, dt):
        if self.phase == "turn":
            out = self.turn.update(fw, dt)
            if self.turn.done:
                self._begin_measure(fw)
            return out
        if self.sampler.full:
            rho = self.sampler.mean_rho()
            if abs(rho - self.rho_exp) < self.tol:
                self.finish(rho=rho, off_deg=self.offsets[self.idx])
            else:
                self._next(fw)
        elif fw.hal.now_ms() - self.t0 > 800:
            self._next(fw)
        return BRAKE


def _solve3(a: list[list[float]], b: list[float]) -> list[float] | None:
    """Giải hệ 3×3 bằng Cramer (đủ cho fit parabol; dễ port C++)."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(a)
    if abs(d) < 1e-12:
        return None
    out = []
    for c in range(3):
        m = [row[:] for row in a]
        for r in range(3):
            m[r][c] = b[r]
        out.append(det(m) / d)
    return out


def tof_fit(samples: list[tuple[float, float, float]]) -> dict:
    """Từ các mẫu (θ, d_l thô, d_r thô) khi quay ±30° trước một mặt phẳng → bù lệch 2 mắt ToF.

    VL53L0X có chùm rộng ~25°: khi |θ−θ0| < ~12° trong chùm luôn có tia vuông góc nên d̄ gần như
    PHẲNG; ra ngoài thì d̄ tăng nhanh. d̄ = (d_l + d_r)/2 đối xứng quanh θ0 (hướng vuông góc).
    1. θ0 = trung điểm 2 mép đoạn phẳng (nơi d̄ vượt d_min + ngưỡng).
    2. Trong đoạn phẳng: (d_r − d_l) ≈ c − b·(θ − θ0)  → fit tuyến tính.
       c = lệch 2 mắt khi vuông góc (lắp lệch trước/sau + offset riêng module) → bù về 0.
       b = khoảng cách 2 mắt hiệu dụng.
    """
    if len(samples) < 40:
        return {"ok": False, "err": "TOO_FEW_SAMPLES", "n": len(samples)}
    # Gom theo ô 1°, lấy trung vị d̄ mỗi ô (chống nhiễu, không phụ thuộc thứ tự mẫu).
    bins: dict[int, list[float]] = {}
    for th, dl, dr in samples:
        bins.setdefault(int(math.floor(math.degrees(th))), []).append(0.5 * (dl + dr))
    keys = sorted(bins)
    prof = [(k + 0.5, sorted(bins[k])[len(bins[k]) // 2]) for k in keys]
    d_min = min(v for _, v in prof)
    thr = d_min + max(10.0, 0.05 * d_min)
    i_min = min(range(len(prof)), key=lambda i: prof[i][1])

    def cross(rng):
        prev = None
        for i in rng:
            if prof[i][1] > thr:
                if prev is None:
                    return None
                (a, va), (b_, vb) = prof[prev], prof[i]
                return a + (thr - va) * (b_ - a) / (vb - va)
            prev = i
        return None
    left = cross(range(i_min, -1, -1))
    right = cross(range(i_min, len(prof)))
    if left is None or right is None:
        return {"ok": False, "err": "PERP_OUT_OF_SCAN", "d_min_mm": d_min}
    th0 = math.radians(0.5 * (left + right))
    half = math.radians(0.5 * abs(right - left))
    win = max(math.radians(3.0), min(math.radians(8.0), half - math.radians(2.0)))
    near = [(th - th0, dr - dl, 0.5 * (dl + dr)) for th, dl, dr in samples if abs(th - th0) <= win]
    if len(near) < 15:
        return {"ok": False, "err": "TOO_FEW_NEAR", "n": len(near)}
    n = len(near)
    mx = sum(x for x, _, _ in near) / n
    my = sum(y for _, y, _ in near) / n
    sxx = sum((x - mx) ** 2 for x, _, _ in near)
    slope = sum((x - mx) * (y - my) for x, y, _ in near) / sxx
    c = my - slope * mx
    resid = math.sqrt(sum((y - (c + slope * x)) ** 2 for x, y, _ in near) / n)
    d_perp = sorted(m for _, _, m in near)[n // 2]
    return {"ok": True, "c_mm": c, "baseline_mm": -slope, "theta0_deg": math.degrees(th0),
            "plateau_deg": math.degrees(2 * half), "d_perp_mm": d_perp, "resid_mm": resid, "n": len(samples)}


class TofCal(Prim):
    """Hiệu chuẩn 2 mắt ToF: đặt xe trước một mặt phẳng RỘNG (tường/hộp ≥ 40 cm, cách ~150–300 mm,
    gần vuông góc ±10°), xe tự quay chậm ±scan_deg (40°) đi rồi về (2 lượt: gấp đôi mẫu, triệt tiêu
    trễ ToF khi đang quay) và fit (tof_fit). Chạy được khi xe IDLE.

    Kết quả: offset mới cho 2 mắt (bù c, chia đều 2 bên; nếu biết wall_mm — khoảng cách tâm trục
    tới mặt phẳng khi vuông góc — thì bù cả offset tuyệt đối) và baseline hiệu dụng.
    """
    name = "TOF_CAL"
    standalone = True

    def start(self, fw):
        cfg = fw.params.active
        self.scan = math.radians(float(self.args.get("scan_deg") or 40.0))
        self.rate = math.radians(float(self.args.get("rate_dps") or 8.0))
        self.base = fw.heading.psi
        self.samples: list[tuple[float, float, float]] = []
        self.off0 = (cfg["range.offset_l_mm"], cfg["range.offset_r_mm"])
        self.turn = Turn(-1, {"heading_deg": math.degrees(self.base - self.scan)})
        self.turn.start(fw)
        self.phase = "to_start"

    def on_range(self, fw, m):
        if self.phase not in ("sweep", "sweep_back"):
            return
        (lm, ls), (rm, rs) = fw.raw_tof["l"], fw.raw_tof["r"]
        cfg = fw.params.active
        if ls == 0 and rs == 0 and cfg["range.min_valid_mm"] <= min(lm, rm) and max(lm, rm) <= cfg["range.max_valid_mm"]:
            self.samples.append((wrap(fw.heading.psi - self.base), lm, rm))

    def update(self, fw, dt):
        if self.phase == "to_start":
            out = self.turn.update(fw, dt)
            if self.turn.done:
                self.phase = "sweep"
            return out
        if self.phase == "sweep":
            if wrap(fw.heading.psi - self.base) >= self.scan:
                self.phase = "sweep_back"
                return BRAKE
            return 0.0, self.rate, False
        if self.phase == "sweep_back":
            if wrap(fw.heading.psi - self.base) <= -self.scan:
                self.phase = "back"
                self.turn = Turn(-1, {"heading_deg": math.degrees(self.base)})
                self.turn.start(fw)
                return BRAKE
            return 0.0, -self.rate, False
        out = self.turn.update(fw, dt)
        if self.turn.done:
            self._finish(fw)
        return out

    def _finish(self, fw):
        cfg = fw.params.active
        r = tof_fit(self.samples)
        if not r["ok"]:
            self.finish(ok=False, err=r["err"], **{k: v for k, v in r.items() if k not in ("ok", "err")})
            return
        # Số đo đã bù = thô + offset. Muốn (r_bù − l_bù) = 0 khi vuông góc: off_r − off_l = −c.
        mean_off = 0.0
        if self.args.get("wall_mm"):
            mean_off = (float(self.args["wall_mm"]) - cfg["range.x_mm"]) - r["d_perp_mm"]
        r.update(offset_l_mm=mean_off + r["c_mm"] / 2, offset_r_mm=mean_off - r["c_mm"] / 2,
                 old_offset_l_mm=self.off0[0], old_offset_r_mm=self.off0[1],
                 dpsi_bias_deg=math.degrees(math.atan2(r["c_mm"] + self.off0[1] - self.off0[0], cfg["range.baseline_mm"])))
        r.pop("ok")
        self.finish(**r)


class CalDrive(Prim):
    """Hiệu chuẩn đi thẳng: chạy thẳng `mm` (giữ hướng gyro) rồi báo quãng encoder đo được.
    Người dùng đo quãng thật → wheel.diameter_mm mới = cũ × thật / encoder."""
    name = "CAL_DRIVE"
    standalone = True

    def start(self, fw):
        self.s0 = fw.odo.s
        self.arc0 = (fw.odo.arc_l, fw.odo.arc_r)
        self.psi0 = fw.heading.psi
        cfg = fw.params.active
        v = float(self.args.get("v_max") or min(cfg["motion.v_cruise"], 150.0))
        self.drive = DriveTo(-1, {"s_end": self.s0 + float(self.args.get("mm", 1000.0)),
                                  "heading_deg": math.degrees(self.psi0), "v_max": v})
        self.drive.start(fw)

    def update(self, fw, dt):
        out = self.drive.update(fw, dt)
        if self.drive.done:
            self.finish(cmd_mm=float(self.args.get("mm", 1000.0)), enc_mm=fw.odo.s - self.s0,
                        arc_l_mm=fw.odo.arc_l - self.arc0[0], arc_r_mm=fw.odo.arc_r - self.arc0[1],
                        yaw_change_deg=math.degrees(wrap(fw.heading.psi - self.psi0)))
        return out


class CalTurn(Prim):
    """Hiệu chuẩn quay: quay tại chỗ `deg` (cộng dồn, quay được 360°/720°) theo gyro.
    Người dùng đo góc thật → imu.gyro_scale_z và wheel.track_eff_mm mới."""
    name = "CAL_TURN"
    standalone = True

    def start(self, fw):
        self.target = math.radians(float(self.args.get("deg", 360.0)))
        self.acc = 0.0
        self.arc0 = (fw.odo.arc_l, fw.odo.arc_r)
        self.w_prev = 0.0
        self.settle = 0.0

    def update(self, fw, dt):
        cfg = fw.params.active
        self.acc += fw.heading.w * dt  # góc cộng dồn (không wrap) theo bộ ước lượng
        e = self.target - self.acc
        tol = math.radians(cfg["motion.turn_tol_deg"])
        if abs(e) < tol:
            self.settle += dt
            if self.settle > 0.3:
                self.finish(cmd_deg=math.degrees(self.target), est_deg=math.degrees(self.acc),
                            arc_l_mm=fw.odo.arc_l - self.arc0[0], arc_r_mm=fw.odo.arc_r - self.arc0[1])
            return BRAKE
        self.settle = 0.0
        wmax = math.radians(cfg["motion.turn_rate_dps"])
        alpha = math.radians(cfg["motion.turn_acc_dps2"])
        w = max(min(wmax, 0.8 * math.sqrt(2 * alpha * abs(e))), turn_w_min(cfg))
        w = min(w, abs(self.w_prev) + alpha * dt)
        self.w_prev = w
        return 0.0, sign(e) * w, False


class CalKs(Prim):
    """Đo vùng chết từng bánh: tăng điện áp 1.5 V/s tới khi bánh quay (→ điện áp BỨT ma sát tĩnh), rồi giảm 3 V/s
    tới khi bánh dừng (→ điện áp GIỮ quay = motor.ks). Chiều tiến rồi chiều lùi. Trả lệnh RAW (điện áp)."""
    name = "CAL_KS"
    standalone = True
    needs_imu = False
    RATE_UP, RATE_DOWN, V_MAX, MOVE, STOP = 1.5, 3.0, 9.0, 15.0, 4.0

    def start(self, fw):
        self.ph = ["UP", "UP"]
        self.V = [0.0, 0.0]
        self.brk = [[0.0, 0.0], [0.0, 0.0]]
        self.stp = [[0.0, 0.0], [0.0, 0.0]]
        self.dir = 1
        self.rest = 0.0
        self.t = 0.0

    def update(self, fw, dt):
        self.t += dt
        if self.t > 30.0:
            self.finish(ok=False, err="TIMEOUT")
            return BRAKE
        vm = (fw.drive.v_meas_l * self.dir, fw.drive.v_meas_r * self.dir)
        di = 0 if self.dir > 0 else 1
        for i in range(2):
            if self.ph[i] == "UP":
                self.V[i] += self.RATE_UP * dt
                if vm[i] > self.MOVE:
                    self.brk[i][di] = self.V[i]
                    self.ph[i] = "DOWN"
                elif self.V[i] > self.V_MAX:
                    self.finish(ok=False, err="NO_MOVE_L" if i == 0 else "NO_MOVE_R")
                    return BRAKE
            elif self.ph[i] == "DOWN":
                self.V[i] -= self.RATE_DOWN * dt
                if vm[i] < self.STOP or self.V[i] <= 0:
                    self.stp[i][di] = max(self.V[i], 0.0)
                    self.ph[i] = "DONE"
        if self.ph == ["DONE", "DONE"]:
            self.rest += dt
            if self.rest < 0.5:
                return BRAKE
            if self.dir > 0:
                self.dir, self.rest, self.ph, self.V = -1, 0.0, ["UP", "UP"], [0.0, 0.0]
                return BRAKE
            res = {}
            for i, s in enumerate("lr"):
                res.update({f"break_{s}_fwd": self.brk[i][0], f"break_{s}_back": self.brk[i][1],
                            f"hold_{s}_fwd": self.stp[i][0], f"hold_{s}_back": self.stp[i][1]})
            self.finish(vbat=fw.vbat, **res)
            return BRAKE
        vl = 0.0 if self.ph[0] == "DONE" else self.dir * self.V[0]
        vr = 0.0 if self.ph[1] == "DONE" else self.dir * self.V[1]
        return RAW(vl, vr)


PRIMS: dict[str, type[Prim]] = {
    c.name: c for c in (DriveTo, Turn, Hold, Follow, Snapshot, Align, Reacquire, TofCal, CalDrive, CalTurn, CalKs,
                                 FollowPlate)
}

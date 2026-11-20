"""Primitive chuyển động — PC chỉ ra lệnh ở mức này (wiki/architecture.md §3.1).

Mỗi primitive: start(fw) → update(fw, dt) trả (v, ω, brake) mỗi chu kỳ 100 Hz,
on_range(fw, meas) khi có mẫu ToF mới. Kết thúc bằng finish() → firmware gửi evt prim_done.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

from .mathutil import clamp, sign, wrap
from .rangeproc import RangeMeas

if TYPE_CHECKING:
    from .robot import VirtualFirmware

BRAKE = (0.0, 0.0, True)


class Prim:
    name = "?"

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
        self.v_prev = max(0.0, fw.drive.v_meas)

    def update(self, fw, dt):
        cfg = fw.params.active
        rem = self.s_end - fw.odo.s
        if rem <= cfg["motion.stop_tol_mm"]:
            self.finish(s=fw.odo.s, yaw_deg=math.degrees(fw.heading.psi), overshoot_mm=-rem)
            return BRAKE
        a, vc, app = cfg["motion.a_max"], cfg["motion.v_corner"], cfg["motion.approach_dist"]
        if rem > app:
            v = min(self.v_max, math.sqrt(vc * vc + 2 * a * (rem - app)))
        else:
            v = max(min(vc, cfg["pid.pos_kp"] * rem), cfg["motor.v_min_reliable"])
        v = min(v, self.v_prev + a * dt)
        self.v_prev = v
        return v, _heading_hold(fw, self.hold, self.hint, dt), False


class Turn(Prim):
    """Quay tại chỗ vòng kín theo hướng (gyro). args: deg (tương đối) hoặc heading_deg (tuyệt đối)."""
    name = "TURN"

    def start(self, fw):
        if self.args.get("heading_deg") is not None:
            tgt = math.radians(float(self.args["heading_deg"]))
        else:
            tgt = fw.heading.psi + math.radians(float(self.args.get("deg", 0.0)))
        self.target = wrap(tgt)
        self.w_prev = 0.0
        self.settle = 0.0

    def update(self, fw, dt):
        cfg = fw.params.active
        e = wrap(self.target - fw.heading.psi)
        tol = math.radians(cfg["motion.turn_tol_deg"])
        if abs(e) < tol:
            self.settle += dt
            if self.settle > 0.15:
                self.finish(yaw_deg=math.degrees(fw.heading.psi), err_deg=math.degrees(e))
            return BRAKE
        self.settle = 0.0
        wmax = math.radians(cfg["motion.turn_rate_dps"])
        alpha = math.radians(cfg["motion.turn_acc_dps2"])
        w = min(wmax, 0.8 * math.sqrt(2 * alpha * abs(e)))
        w = max(w, math.radians(6.0))
        w = min(w, abs(self.w_prev) + alpha * dt)
        self.w_prev = w
        return 0.0, sign(e) * w, False


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
        self.v_prev = max(0.0, fw.drive.v_meas)
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
        v = kf.vL + cfg["follow.kp_g"] * e + self.integ
        a = cfg["motion.a_max"]
        v = clamp(v, 0.0, self.v_max)
        v = min(v, self.v_prev + a * dt)
        if self.s_limit is not None:
            rem = float(self.s_limit) - fw.odo.s
            if rem <= cfg["motion.stop_tol_mm"]:
                self.finish(s=fw.odo.s, gap=kf.g)
                return BRAKE
            v = min(v, math.sqrt(2 * a * rem))
        if kf.g < self.g_min:
            v = 0.0
            self.integ = min(self.integ, 0.0)
        self.v_prev = v
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


class Snapshot(Prim):
    """Đứng yên, lấy trung bình N mẫu ToF → ρ, Δψ (mốc neo ở góc cua)."""
    name = "SNAPSHOT"

    def start(self, fw):
        cfg = fw.params.active
        self.n = int(self.args.get("n") or cfg["corner.snapshot_n"])
        self.samples: list[tuple[float, float]] = []
        self.t0 = fw.hal.now_ms()

    def on_range(self, fw, m):
        if m.valid and m.dpsi is not None:
            self.samples.append((m.rho, m.dpsi))

    def update(self, fw, dt):
        if len(self.samples) >= self.n:
            rhos = [r for r, _ in self.samples]
            mean = sum(rhos) / len(rhos)
            std = math.sqrt(sum((r - mean) ** 2 for r in rhos) / len(rhos))
            dpsi = sum(d for _, d in self.samples) / len(self.samples)
            self.finish(rho=mean, std=std, dpsi_deg=math.degrees(dpsi), s=fw.odo.s, n=len(rhos))
        elif fw.hal.now_ms() - self.t0 > 3000:
            self.finish(ok=False, err="NO_TARGET", n=len(self.samples))
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


PRIMS: dict[str, type[Prim]] = {
    c.name: c for c in (DriveTo, Turn, Hold, Follow, Snapshot, Align, Reacquire)
}

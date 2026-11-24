from __future__ import annotations


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class JerkRamp:
    """Giới hạn đạo hàm gia tốc để primitive tăng/giảm tốc êm hơn."""

    def __init__(self):
        self.v = 0.0
        self.a = 0.0

    def reset(self, v0: float = 0.0) -> None:
        self.v = v0
        self.a = 0.0

    def acceleration(self) -> float:
        return self.a

    def velocity(self) -> float:
        return self.v

    def step(self, target: float, accel_limit: float, dt: float) -> float:
        if dt <= 0 or accel_limit <= 0:
            return self.v
        desired_a = _clamp((target - self.v) / dt, -accel_limit, accel_limit)
        jerk = 4 * accel_limit
        self.a = _clamp(desired_a, self.a - jerk * dt, self.a + jerk * dt)
        previous = self.v
        self.v += self.a * dt
        if target != previous and (target - previous) * (target - self.v) <= 0:
            self.v = target
            self.a = 0.0
        return self.v

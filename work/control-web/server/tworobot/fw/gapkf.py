"""Kalman filter 2 trạng thái cho gap: x = [g, v_L] (wiki/algorithms.md §4.2)."""
from __future__ import annotations


class GapKF:
    def __init__(self):
        self.g = 0.0
        self.vL = 0.0
        self.P = [[1e4, 0.0], [0.0, 1e4]]
        self.rejects = 0
        self.active = False

    def reset(self, g: float, vL: float = 0.0, sigma_g: float = 10.0, sigma_v: float = 150.0) -> None:
        self.g, self.vL = g, vL
        self.P = [[sigma_g ** 2, 0.0], [0.0, sigma_v ** 2]]
        self.rejects = 0
        self.active = True

    def predict(self, vF: float, dt: float, q_a: float) -> None:
        self.g += (self.vL - vF) * dt
        p = self.P
        # P = F P F^T, F = [[1, dt], [0, 1]]
        p00 = p[0][0] + dt * (p[1][0] + p[0][1]) + dt * dt * p[1][1]
        p01 = p[0][1] + dt * p[1][1]
        p10 = p[1][0] + dt * p[1][1]
        p11 = p[1][1]
        q = q_a * q_a
        self.P = [[p00 + q * dt ** 4 / 4, p01 + q * dt ** 3 / 2],
                  [p10 + q * dt ** 3 / 2, p11 + q * dt * dt]]

    def update_g(self, z: float, sigma: float, gate_chi2: float) -> bool:
        """Đo gap trực tiếp (ToF). Trả False nếu bị gating loại."""
        p = self.P
        s = p[0][0] + sigma * sigma
        nu = z - self.g
        if nu * nu / s > gate_chi2:
            self.rejects += 1
            return False
        self.rejects = 0
        k0, k1 = p[0][0] / s, p[1][0] / s
        self.g += k0 * nu
        self.vL += k1 * nu
        self.P = [[(1 - k0) * p[0][0], (1 - k0) * p[0][1]],
                  [p[1][0] - k1 * p[0][0], p[1][1] - k1 * p[0][1]]]
        return True

    def update_v(self, z: float, sigma: float) -> None:
        """Đo vận tốc leader (relay qua PC)."""
        p = self.P
        s = p[1][1] + sigma * sigma
        nu = z - self.vL
        k0, k1 = p[0][1] / s, p[1][1] / s
        self.g += k0 * nu
        self.vL += k1 * nu
        self.P = [[p[0][0] - k0 * p[1][0], p[0][1] - k0 * p[1][1]],
                  [(1 - k1) * p[1][0], (1 - k1) * p[1][1]]]

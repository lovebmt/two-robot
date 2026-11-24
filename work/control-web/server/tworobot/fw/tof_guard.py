"""Phát hiện một hoặc hai mắt ToF bất thường khi follower đang FOLLOW."""
from __future__ import annotations

from collections import deque


class TofAnomalyGuard:
    def __init__(self):
        self._hist_l: deque[float] = deque(maxlen=5)
        self._hist_r: deque[float] = deque(maxlen=5)
        self._streak_l = 0
        self._streak_r = 0
        self._armed = False

    def reset(self) -> None:
        self._hist_l.clear()
        self._hist_r.clear()
        self._streak_l = 0
        self._streak_r = 0
        self._armed = False

    @staticmethod
    def _valid(mm: float, st: int, cfg: dict) -> bool:
        return st == 0 and cfg["range.min_valid_mm"] <= mm <= cfg["range.max_valid_mm"]

    @staticmethod
    def _median(hist: deque[float]) -> float:
        values = sorted(hist)
        return values[len(values) // 2]

    def _side(self, mm: float, st: int, hist: deque[float], streak: int, cfg: dict) -> int:
        if not self._valid(mm, st, cfg):
            return streak + 1 if hist else 0
        jump = self._median(hist) - mm if len(hist) >= 3 else 0.0
        streak = streak + 1 if jump >= cfg["safety.tof_anomaly_jump_mm"] else 0
        hist.append(mm)
        return streak

    def update(self, l_mm: float, l_st: int, r_mm: float, r_st: int, cfg: dict) -> bool:
        self._streak_l = self._side(l_mm, l_st, self._hist_l, self._streak_l, cfg)
        self._streak_r = self._side(r_mm, r_st, self._hist_r, self._streak_r, cfg)
        if not self._armed:
            self._armed = len(self._hist_l) >= 3 and len(self._hist_r) >= 3
            return False
        need = int(cfg["safety.tof_anomaly_samples"])
        if self._streak_l >= need or self._streak_r >= need:
            self.reset()
            return True
        return False

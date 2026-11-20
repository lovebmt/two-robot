"""Xử lý 2 ToF → gap tâm–tâm ρ, góc lệch tương đối Δψ (wiki/algorithms.md §4.1).

Quy ước dấu: dpsi = yaw_leader − yaw_follower = atan2(d_r − d_l, b).
Mắt trái ở y = +b/2 (bên trái xe). Leader xoay ngược chiều kim đồng hồ ⇒ mặt sau
lại gần mắt trái ⇒ d_l < d_r ⇒ dpsi > 0 ⇒ follower cần ω > 0 để song song.
edge = +1 khi chỉ mắt trái thấy leader (leader lệch trái), −1 khi chỉ mắt phải.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


@dataclass
class RangeMeas:
    valid: bool
    rho: float | None
    dpsi: float | None
    dl: float | None
    dr: float | None
    edge: int
    t_ms: int = 0


class RangeProcessor:
    def __init__(self):
        self._hl: deque[float] = deque(maxlen=3)
        self._hr: deque[float] = deque(maxlen=3)

    def reset(self) -> None:
        self._hl.clear()
        self._hr.clear()

    @staticmethod
    def _side(mm: float, st: int, off: float, cfg: dict, hist: deque) -> float | None:
        if st != 0 or not (cfg["range.min_valid_mm"] <= mm <= cfg["range.max_valid_mm"]):
            hist.clear()
            return None
        hist.append(mm + off)
        return sorted(hist)[len(hist) // 2]

    def process(self, l_mm: float, l_st: int, r_mm: float, r_st: int, cfg: dict, leader_rear: float) -> RangeMeas:
        dl = self._side(l_mm, l_st, cfg["range.offset_l_mm"], cfg, self._hl)
        dr = self._side(r_mm, r_st, cfg["range.offset_r_mm"], cfg, self._hr)
        xs, b = cfg["range.x_mm"], cfg["range.baseline_mm"]
        if dl is not None and dr is not None and abs(dl - dr) <= cfg["range.max_diff_mm"]:
            dpsi = math.atan2(dr - dl, b)
            rho = xs + 0.5 * (dl + dr) * math.cos(dpsi) + leader_rear
            return RangeMeas(True, rho, dpsi, dl, dr, 0)
        if dl is not None and (dr is None or dl < dr):
            return RangeMeas(True, xs + dl + leader_rear, None, dl, dr, +1)
        if dr is not None:
            return RangeMeas(True, xs + dr + leader_rear, None, dl, dr, -1)
        return RangeMeas(False, None, None, dl, dr, 0)

"""Kịch bản hồi quy: chạy trọn đường trong mô phỏng (nhanh hơn realtime)."""
import asyncio

import pytest

from tworobot.sim import batch

MISMATCH = {"leader": {"motor_scale_r": 0.85, "fric_scale": 1.3},
            "follower": {"motor_scale_l": 0.88, "gyro_bias_dps": 1.0, "slip_turn": 0.06}}


@pytest.mark.parametrize("path", ["line", "L", "rect"])
def test_run_completes_without_collision(path):
    r = asyncio.run(batch.run_scenario(batch.PRESETS[path], seed=2, mismatch=MISMATCH))
    assert r["ok"], r.get("error")
    assert r["collisions"] == 0
    assert r["gap_rmse_mm"] is not None and r["gap_rmse_mm"] < 20  # N1: ±2 cm
    assert all(e < 30 for e in r["corner_err_mm"])  # nghiệm thu: ±3 cm


def test_tof_block_triggers_safe_stop():
    """Che ToF lâu hơn lost_timeout khi đang bám → follower dừng, coordinator báo lỗi."""
    async def scenario():
        orig = batch.SimEngine.step
        state = {"done": False}

        def step(self):
            if not state["done"] and self.t > 6.0:
                self.inject("tof_block", "sim-b", side="l", dur=3.0)
                self.inject("tof_block", "sim-b", side="r", dur=3.0)
                state["done"] = True
            orig(self)
        batch.SimEngine.step = step
        try:
            return await batch.run_scenario(batch.PRESETS["line"], seed=1)
        finally:
            batch.SimEngine.step = orig
    r = asyncio.run(scenario())
    assert not r["ok"]
    assert "LEADER_LOST" in (r["error"] or "") or "TOO" in (r["error"] or "")
    assert r["collisions"] == 0


@pytest.mark.parametrize("path", ["L", "zigzag"])
def test_solo_run_follows_drawn_path(path):
    """Chế độ 1 xe: leader chạy hết đường vẽ, dừng-quay tại góc gần đỉnh."""
    r = asyncio.run(batch.run_scenario(batch.PRESETS[path], seed=2, mismatch=MISMATCH, solo=True))
    assert r["ok"], r.get("error")
    assert r["collisions"] == 0
    assert all(e < 30 for e in r["leader_corner_err_mm"])
    assert r["leader_end_err_mm"] < 60

"""SimEngine: bước vật lý cố định 2 ms, lập lịch task firmware đúng tần số,
deterministic theo seed. Chạy realtime (có hệ số thời gian) hoặc nhanh (batch).
"""
from __future__ import annotations

import asyncio
import math
import random
import time
from dataclasses import dataclass, field
from pathlib import Path

from ..fw.robot import VirtualFirmware
from .hal_sim import SimHal
from .world import Body, FrictionZone, World, physical_from_cfg

PHYS_DT = 0.002
CONTROL_DT = 0.010
TOF_DT = 1 / 30


@dataclass
class SimRobot:
    id: str
    body: Body
    fw: VirtualFirmware
    hal: SimHal
    mismatch: dict = field(default_factory=dict)
    link_alive: bool = True
    link_down_until: float = 0.0
    ctrl_acc: float = 0.0
    tof_acc: float = 0.0
    collisions: int = 0


class SimEngine:
    def __init__(self, width_mm: float, height_mm: float, seed: int = 1, state_dir: Path | None = None):
        self.rng = random.Random(seed)
        self.seed = seed
        self.t = 0.0
        self.world = World(width_mm, height_mm)
        self.robots: dict[str, SimRobot] = {}
        self.state_dir = state_dir
        self.running = True
        self.scale = 1.0
        self.collision_now: set[str] = set()
        self.collision_events = 0
        self._stop = False
        self._debt = 0.0

    # ------------------------------------------------------------ robots
    def add_robot(self, rid: str, name: str, x: float, y: float, th: float,
                  mismatch: dict | None = None) -> SimRobot:
        rng = random.Random(self.rng.random())
        # Body tạm để HAL đọc flash → firmware tải config → dựng vật lý thật từ config.
        from .world import DEFAULT_MISMATCH  # noqa: PLC0415
        from .. import params as P  # noqa: PLC0415
        body = Body(rid, physical_from_cfg(P.defaults(), DEFAULT_MISMATCH), x, y, th)
        hal = SimHal(self, body, self.state_dir, rng)
        fw = VirtualFirmware(rid, name, hal)
        if "CFG_DEFAULT" in fw.params.flags:
            fw.params.save()  # robot ảo "xuất xưởng" với cấu hình mặc định đã lưu flash
        r = SimRobot(rid, body, fw, hal, dict(mismatch or {}))
        body.p = physical_from_cfg(fw.params.active, r.mismatch)
        self.world.bodies[rid] = body
        self.robots[rid] = r
        return r

    def rebuild_physics(self, rid: str) -> None:
        r = self.robots[rid]
        r.body.p = physical_from_cfg(r.fw.params.view(), r.mismatch)

    def set_mismatch(self, rid: str, patch: dict) -> dict:
        r = self.robots[rid]
        r.mismatch.update(patch)
        self.rebuild_physics(rid)
        return r.mismatch

    def place(self, rid: str, x: float, y: float, yaw_deg: float) -> None:
        r = self.robots[rid]
        self.rebuild_physics(rid)
        r.body.place(x, y, math.radians(yaw_deg))

    def set_zones(self, zones: list[dict]) -> None:
        self.world.zones = [FrictionZone(z["x0"], z["y0"], z["x1"], z["y1"], z["factor"]) for z in zones]

    # ------------------------------------------------------------ faults
    def inject(self, kind: str, rid: str, **kw) -> None:
        r = self.robots[rid]
        dur = float(kw.get("dur", 1.0))
        if kind == "tof_block":
            r.body.tof_block[kw.get("side", "l")] = self.t + dur
        elif kind == "link_drop":
            r.link_down_until = self.t + dur
        elif kind == "push":
            dx, dy = float(kw.get("dx", 0.0)), float(kw.get("dy", 30.0))
            c, s = math.cos(r.body.th), math.sin(r.body.th)
            r.body.x += dx * c - dy * s
            r.body.y += dx * s + dy * c
            r.body.th += math.radians(float(kw.get("dth_deg", 3.0)))
        elif kind == "caster":
            r.body.caster_stuck_until = self.t + dur
        else:
            raise ValueError(f"fault không hỗ trợ: {kind}")

    # ------------------------------------------------------------ stepping
    def truth(self, rid: str) -> dict:
        b = self.robots[rid].body
        return {"x": b.x, "y": b.y, "yaw": b.th, "v": b.v, "w": b.w,
                "collide": rid in self.collision_now}

    def step(self) -> None:
        dt = PHYS_DT
        for r in self.robots.values():
            r.body.step(dt, self.world.fric_factor(r.body.x, r.body.y), self.t, r.hal.rng)
        self.t += dt
        hits = self.world.collisions()
        now_set = {i for pair in hits for i in pair}
        if now_set - self.collision_now:
            self.collision_events += 1
            for rid in now_set - self.collision_now:
                self.robots[rid].collisions += 1
        self.collision_now = now_set
        for r in self.robots.values():
            r.link_alive = self.t >= r.link_down_until
            r.ctrl_acc += dt
            while r.ctrl_acc >= CONTROL_DT - 1e-9:
                r.ctrl_acc -= CONTROL_DT
                if r.link_alive:
                    r.fw.last_hb_ms = r.hal.now_ms()  # heartbeat của SimLink (theo thời gian sim)
                r.fw.tick(CONTROL_DT)
            r.tof_acc += dt
            if r.tof_acc >= TOF_DT:
                r.tof_acc -= TOF_DT
                lm, ls = self.world.tof(r.body, "l", self.t, r.hal.rng)
                rm, rs = self.world.tof(r.body, "r", self.t, r.hal.rng)
                r.fw.on_range(lm, ls, rm, rs)

    def advance(self, seconds: float) -> None:
        n = int(round(seconds / PHYS_DT))
        for _ in range(n):
            self.step()

    async def run_realtime(self) -> None:
        last = time.perf_counter()
        while not self._stop:
            await asyncio.sleep(0.01)
            now = time.perf_counter()
            real_dt = min(now - last, 0.1)
            last = now
            if not self.running:
                continue
            self._debt += real_dt * self.scale
            n = int(self._debt / PHYS_DT)
            self._debt -= n * PHYS_DT
            for _ in range(min(n, 4000)):
                self.step()

    async def run_fast(self, chunk_s: float = 0.05) -> None:
        """Chạy nhanh nhất có thể, nhường event loop sau mỗi chunk (dùng cho batch/test)."""
        while not self._stop:
            if self.running:
                self.advance(chunk_s)
            await asyncio.sleep(0)

    def stop(self) -> None:
        self._stop = True

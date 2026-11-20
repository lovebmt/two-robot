"""Chạy kịch bản không UI, nhanh hơn realtime, xuất metric (wiki/simulation.md §4, §7).

    tworobot-batch --path L --seeds 3 --mismatch '{"leader":{"motor_scale_r":0.85}}'
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math

from .. import params as P
from ..coordinator import Coordinator
from ..fw.mathutil import wrap
from ..links import SimLink
from ..planner import plan_path
from ..registry import Registry, RobotSession
from .engine import SimEngine

PRESETS = {
    "line": [[5, 4], [28, 4]],
    "L": [[5, 3], [21, 3], [21, 14]],
    "rect": [[6, 3], [26, 3], [26, 13], [6, 13], [6, 6]],
    "zigzag": [[5, 2], [12, 2], [12, 7], [20, 7], [20, 12], [28, 12]],
}
GRID = {"cell_mm": 100, "cols": 32, "rows": 18}


async def run_scenario(points, grid=GRID, seed: int = 1, mismatch: dict | None = None,
                       shared_patch: dict | None = None, robot_patch: dict | None = None,
                       idle_before_s: float = 1.0, max_t: float = 600.0,
                       start_offset: dict | None = None) -> dict:
    """start_offset: {'leader'|'follower': {'dx','dy' (mm, theo hệ xe), 'dyaw' (độ)}} — đặt xe lệch so với
    vị trí xuất phát (firmware không biết), giống người đặt xe không chuẩn."""
    mismatch = mismatch or {}
    eng = SimEngine(grid["cols"] * grid["cell_mm"], grid["rows"] * grid["cell_mm"], seed=seed, state_dir=None)
    reg = Registry()
    shared = P.shared_defaults()
    shared.update(shared_patch or {})
    sessions = {}
    for idx, (rid, role) in enumerate((("sim-a", "leader"), ("sim-b", "follower"))):
        r = eng.add_robot(rid, rid, 300 + 600 * idx, 300, 0, mismatch.get(role))
        s = RobotSession(rid, rid, SimLink(eng, r), "SIM")
        reg.add(s)
        await s.link.request("PUT", "/api/config", {"patch": {**shared, **(robot_patch or {})}})
        eng.advance(0.02)  # swap tham số ở chu kỳ control kế tiếp
        await reg.refresh(s)
        await reg.set_role(rid, role)
        sessions[role] = s

    L, F = sessions["leader"], sessions["follower"]
    form, ferr, _ = P.formation(reg.cfg_params(L), reg.cfg_params(F), shared)
    plan = plan_path(points, grid, form, shared, {"rear": reg.cfg_params(F)["body.axle_to_rear_mm"]})
    if ferr or not plan["ok"]:
        return {"ok": False, "errors": plan["errors"] + list(ferr.values())}
    path = plan["path"]
    st, fs = path["start"], path["follower"]["start"]
    def place(sess, pose, role):
        o = (start_offset or {}).get(role, {})
        th = math.radians(pose["yaw_deg"])
        dx, dy = o.get("dx", 0.0), o.get("dy", 0.0)
        eng.place(sess.id, pose["x"] + dx * math.cos(th) - dy * math.sin(th),
                  pose["y"] + dx * math.sin(th) + dy * math.cos(th), pose["yaw_deg"] + o.get("dyaw", 0.0))
    place(L, st, "leader")
    place(F, fs, "follower")
    eng.collision_events = 0
    for s in (L, F):
        await s.link.request("PUT", "/api/path", path)
    eng.advance(idle_before_s)  # xe vừa bật nguồn, nằm chờ trên sân (chip nóng dần)

    marks: dict[str, list] = {"L_turn": [], "F_turn": []}
    gap_err: list[float] = []
    min_gap = [1e9]
    yaw_err = {"leader": 0.0, "follower": 0.0}

    def on_mark(tag, k):
        b = eng.robots[(L if tag == "L_turn" else F).id].body
        marks[tag].append((b.x, b.y))

    coord = Coordinator(clock=lambda: eng.t, poll=0.0, on_mark=on_mark)
    reg.on_event = coord.on_event

    def on_tel(sess, msg):
        if coord.phase == "RUN" and msg.get("truth"):
            e = abs(math.degrees(wrap(msg["est"]["yaw"] - msg["truth"]["yaw"])))
            role = "leader" if sess is L else "follower"
            yaw_err[role] = max(yaw_err[role], e)
        if sess is F and msg.get("prim") and msg["prim"]["name"] == "FOLLOW":
            bl, bf = eng.robots[L.id].body, eng.robots[F.id].body
            g = math.hypot(bl.x - bf.x, bl.y - bf.y)
            min_gap[0] = min(min_gap[0], g)
            if msg["gap"]["target"] and abs(bl.v) > 0.8 * shared["motion.v_cruise"] and abs(bf.v) > 0.8 * shared["motion.v_cruise"]:
                gap_err.append(g - msg["gap"]["target"])
    reg.on_tel = on_tel

    coord.start(path, L, F, shared)
    runner = asyncio.get_running_loop().create_task(eng.run_fast(0.05))
    try:
        while coord.state == "RUNNING" and eng.t < max_t:
            await asyncio.sleep(0)
    finally:
        eng.stop()
        await runner
    if coord.state == "RUNNING":
        await coord.stop()

    corner_err = [math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(marks["L_turn"], marks["F_turn"])]
    end = path["segments"][-1]["b"]
    bl = eng.robots[L.id].body
    calib = [e["msg"] for e in coord.log if "calib IMU" in e["msg"]]
    rms = math.sqrt(sum(e * e for e in gap_err) / len(gap_err)) if gap_err else None
    return {
        "ok": coord.state == "DONE", "state": coord.state, "error": coord.error,
        "sim_time_s": round(eng.t, 1), "collisions": eng.collision_events,
        "gap_rmse_mm": None if rms is None else round(rms, 1),
        "gap_samples": len(gap_err),
        "min_center_gap_mm": round(min_gap[0], 1),
        "corner_err_mm": [round(e, 1) for e in corner_err],
        "yaw_est_err_max_deg": {k: round(v, 2) for k, v in yaw_err.items()},
        "leader_end_err_mm": round(math.hypot(bl.x - end[0], bl.y - end[1]), 1),
        "gyro_bias_true_dps": {r: round(math.degrees(eng.robots[s.id].body.gyro_bias_true()), 2)
                               for r, s in (("leader", L), ("follower", F))},
        "calib": calib,
        "log": coord.log,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Chạy kịch bản mô phỏng 2 robot (không UI)")
    ap.add_argument("--path", default="rect", choices=sorted(PRESETS))
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--mismatch", default="{}", help='JSON {"leader": {...}, "follower": {...}}')
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    mm = json.loads(a.mismatch)
    for seed in range(1, a.seeds + 1):
        res = asyncio.run(run_scenario(PRESETS[a.path], seed=seed, mismatch=mm))
        if not a.verbose:
            res.pop("log", None)
        print(json.dumps({"seed": seed, **res}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()

"""Control Server (FastAPI): REST cho UI, WebSocket /ws/ui đẩy TelemetryFrame 20 Hz.

Server không phân biệt robot ảo hay thật (chỉ dùng RobotLink). Chạy:
    tworobot-server            # http://127.0.0.1:8080
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from collections import deque
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path

from fastapi import Body, FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from . import __version__
from . import params as P
from .coordinator import Coordinator
from .discovery import start_listener
from .links import LinkError, NetLink, SimLink
from .planner import plan_path
from .registry import Registry, RobotSession
from .sim.engine import SimEngine

ROOT = Path(__file__).resolve().parents[2]
WEB_DIR = ROOT / "web"
STATE_DIR = Path(os.environ.get("TWOROBOT_STATE", ROOT / ".simstate"))
LOG_DIR = Path(os.environ.get("TWOROBOT_LOGS", ROOT / "logs"))
log = logging.getLogger("tworobot")

SIM_ROBOTS = (("sim-a", "Robot ảo A"), ("sim-b", "Robot ảo B"))


class Server:
    def __init__(self):
        self.reg = Registry()
        self.reg.on_event = self._on_event
        self.reg.on_tel = lambda s, m: None
        self.coord = Coordinator(clock=self.clock)
        self.sim: SimEngine | None = None
        self.sim_task: asyncio.Task | None = None
        self.shared = P.shared_defaults()
        self.grid = {"cell_mm": 100, "cols": 30, "rows": 18}
        self.clients: set[WebSocket] = set()
        self.events: deque = deque(maxlen=300)
        self._pending_events: list[dict] = []
        self.rec_file = None
        self.rec_name: str | None = None
        self.replay_task: asyncio.Task | None = None
        self.replay_info: dict | None = None
        self.last_path: dict | None = None
        self._bg: list[asyncio.Task] = []
        self._udp = None

    # ------------------------------------------------------------ time
    def clock(self) -> float:
        if self.sim is not None and all(s.source == "SIM" for s in self.reg.sessions.values()):
            return self.sim.t
        return time.monotonic()

    # ------------------------------------------------------------ events
    def _on_event(self, sess: RobotSession, msg: dict) -> None:
        self.coord.on_event(sess, msg)
        if msg.get("t") in ("ack",):
            return
        e = {"t": round(self.clock(), 2), "robot": sess.name, **{k: v for k, v in msg.items() if k != "t"},
             "kind": msg.get("e") or msg.get("t")}
        self.events.append(e)
        self._pending_events.append(e)

    def note(self, text: str, level: str = "info") -> None:
        e = {"t": round(self.clock(), 2), "robot": "PC", "kind": level, "msg": text}
        self.events.append(e)
        self._pending_events.append(e)

    # ------------------------------------------------------------ frame
    def source(self) -> str:
        if self.replay_task and not self.replay_task.done():
            return "REPLAY"
        srcs = {s.source for s in self.reg.sessions.values()}
        if not srcs:
            return "NONE"
        return srcs.pop() if len(srcs) == 1 else "MIXED"

    def sim_status(self) -> dict | None:
        if self.sim is None:
            return None
        return {"t": round(self.sim.t, 3), "running": self.sim.running, "scale": self.sim.scale,
                "seed": self.sim.seed, "collisions": self.sim.collision_events,
                "mismatch": {rid: r.mismatch for rid, r in self.sim.robots.items()},
                "zones": [z.__dict__ for z in self.sim.world.zones]}

    def frame(self) -> dict:
        robots = []
        for s in self.reg.sessions.values():
            item = {"id": s.id, "name": s.name, "source": s.source, "role": s.role,
                    "online": s.summary()["online"]}
            if s.last_tel:
                item.update({k: v for k, v in s.last_tel.items() if k not in ("t", "id", "role")})
            robots.append(item)
        ev, self._pending_events = self._pending_events, []
        return {"type": "frame", "t": round(self.clock(), 3), "source": self.source(), "robots": robots,
                "coord": self.coord.status(), "sim": self.sim_status(), "events": ev,
                "recording": self.rec_name, "grid": self.grid,
                "path": self.last_path}

    async def broadcaster(self) -> None:
        while True:
            await asyncio.sleep(0.05)
            if self.replay_task and not self.replay_task.done():
                continue
            if not self.clients and not self.rec_file:
                self._pending_events.clear()
                continue
            f = self.frame()
            if self.rec_file:
                rec = dict(f)
                rec.pop("path", None)
                self.rec_file.write(json.dumps(rec, ensure_ascii=False) + "\n")
            await self.broadcast(f)

    async def broadcast(self, msg: dict) -> None:
        if not self.clients:
            return
        data = json.dumps(msg, ensure_ascii=False)
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_text(data)
            except Exception:  # noqa: BLE001
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def heartbeat(self) -> None:
        """Heartbeat 5 Hz cho robot thật (robot ảo nhận heartbeat theo thời gian sim)."""
        while True:
            await asyncio.sleep(0.2)
            for s in list(self.reg.sessions.values()):
                if s.source == "REAL":
                    s.link.post({"t": "hb"})

    # ------------------------------------------------------------ sim
    async def sim_start(self, seed: int = 1) -> None:
        if self.sim is not None:
            return
        w = self.grid["cols"] * self.grid["cell_mm"]
        h = self.grid["rows"] * self.grid["cell_mm"]
        self.sim = SimEngine(w, h, seed=seed, state_dir=STATE_DIR)
        for k, (rid, name) in enumerate(SIM_ROBOTS):
            r = self.sim.add_robot(rid, name, 300, 300 + 400 * k, 0.0,
                                   {"motor_scale_r": 0.92} if k == 0 else {"motor_scale_l": 0.94})
            sess = RobotSession(rid, name, SimLink(self.sim, r), "SIM")
            self.reg.add(sess)
            await sess.link.request("PUT", "/api/config", {"patch": self.shared})
        self.sim.advance(0.02)
        for rid, _ in SIM_ROBOTS:
            await self.reg.refresh(self.reg.get(rid))
        if self.reg.by_role("leader") is None and self.reg.by_role("follower") is None:
            await self.reg.set_role("sim-a", "leader")
            await self.reg.set_role("sim-b", "follower")
        self.sim_task = asyncio.get_running_loop().create_task(self.sim.run_realtime())
        self.note("Đã khởi động mô phỏng 2 robot ảo")

    async def sim_stop(self) -> None:
        if self.sim is None:
            return
        await self.coord.stop()
        self.sim.stop()
        if self.sim_task:
            await self.sim_task
        for rid, _ in SIM_ROBOTS:
            self.reg.remove(rid)
        self.sim, self.sim_task = None, None
        self.note("Đã tắt mô phỏng")

    # ------------------------------------------------------------ real robots
    def on_beacon(self, msg: dict, ip: str) -> None:
        rid = str(msg["id"])
        if rid in self.reg.sessions:
            s = self.reg.sessions[rid]
            s.last_seen = time.monotonic()
            s.ip = ip
            return
        s = RobotSession(rid, msg.get("name", rid), NetLink(ip), "REAL", role=msg.get("role"), ip=ip)
        s.last_seen = time.monotonic()
        s.info = {"fw": msg.get("fw")}
        self.reg.add(s)
        self.note(f"Phát hiện robot thật {s.name} @ {ip}")

    # ------------------------------------------------------------ replay
    async def replay(self, path: Path, speed: float) -> None:
        frames = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        self.replay_info = {"file": path.name, "n": len(frames), "i": 0}
        prev_t = None
        for i, f in enumerate(frames):
            self.replay_info["i"] = i
            if prev_t is not None:
                await asyncio.sleep(max(0.0, min(1.0, (f["t"] - prev_t) / max(speed, 0.05))))
            prev_t = f["t"]
            f["source"] = "REPLAY"
            f["replay"] = dict(self.replay_info)
            f["path"] = self.last_path
            await self.broadcast(f)
        self.replay_info = None


S = Server()


@asynccontextmanager
async def lifespan(app: FastAPI):
    S._bg = [asyncio.create_task(S.broadcaster()), asyncio.create_task(S.heartbeat())]
    S._udp = await start_listener(S.on_beacon)
    yield
    for t in S._bg:
        t.cancel()
    if S._udp:
        S._udp.close()
    await S.sim_stop()


app = FastAPI(title="two-robot Web Control", version=__version__, lifespan=lifespan)


def _sess(rid: str) -> RobotSession:
    try:
        return S.reg.get(rid)
    except KeyError:
        raise HTTPException(404, f"không có robot {rid}") from None


async def _req(sess: RobotSession, method: str, path: str, body: dict | None = None) -> dict:
    try:
        code, res = await sess.link.request(method, path, body)
    except LinkError as e:
        raise HTTPException(502, f"{sess.name}: {e}") from None
    if code >= 400:
        raise HTTPException(code, res)
    return res


# ---------------------------------------------------------------- basic
@app.get("/api/schema")
def get_schema():
    return P.SCHEMA


@app.get("/api/state")
def get_state():
    return {"version": __version__, "robots": [s.summary() for s in S.reg.sessions.values()],
            "sim": S.sim_status(), "coord": S.coord.status(), "shared": S.shared, "grid": S.grid,
            "recording": S.rec_name}


@app.get("/api/robots")
def robots():
    return [s.summary() for s in S.reg.sessions.values()]


@app.put("/api/robots/{rid}/role")
async def set_role(rid: str, body: dict = Body(...)):
    _sess(rid)
    if S.coord.state == "RUNNING":
        raise HTTPException(409, "đang chạy")
    try:
        await S.reg.set_role(rid, body.get("role"))
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    return {"ok": True}


@app.post("/api/robots/{rid}/identify")
async def identify(rid: str):
    return await _req(_sess(rid), "POST", "/api/identify")


@app.post("/api/robots/{rid}/jog")
async def jog(rid: str, body: dict = Body(...)):
    s = _sess(rid)
    try:
        await s.link.send({"t": "cmd", "op": "jog", "v": float(body.get("v", 0)), "w": float(body.get("w", 0))})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    return {"ok": True}


@app.post("/api/robots/{rid}/reset")
async def reset_robot(rid: str):
    await _sess(rid).link.send({"t": "cmd", "op": "reset"})
    return {"ok": True}


# ---------------------------------------------------------------- config
@app.get("/api/robots/{rid}/config")
async def get_config(rid: str):
    s = _sess(rid)
    res = await _req(s, "GET", "/api/config")
    s.config = res
    return res


@app.put("/api/robots/{rid}/config")
async def put_config(rid: str, body: dict = Body(...)):
    s = _sess(rid)
    try:
        code, res = await s.link.request("PUT", "/api/config",
                                         {"patch": body.get("patch", {}), "persist": bool(body.get("persist"))})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    if code == 200:
        s.config = await _req(s, "GET", "/api/config")
        S.note(f"{s.name}: áp dụng {len(body.get('patch', {}))} tham số (rev {res.get('cfg_rev')})")
    return {"status": code, **res}


@app.post("/api/robots/{rid}/config/{action}")
async def config_action(rid: str, action: str):
    if action not in ("save", "reload", "factory_reset"):
        raise HTTPException(404, "action không hợp lệ")
    s = _sess(rid)
    res = await _req(s, "POST", f"/api/config/{action}")
    s.config = await _req(s, "GET", "/api/config")
    S.note(f"{s.name}: config {action}")
    return res


@app.get("/api/shared")
def get_shared():
    return S.shared


@app.put("/api/shared")
async def put_shared(body: dict = Body(...)):
    """Tham số chung 2 xe: validate trên PC rồi đẩy xuống mọi robot."""
    patch = body.get("patch", {})
    errors = {}
    new = dict(S.shared)
    for k, v in patch.items():
        if k not in P.SHARED_KEYS:
            errors[k] = "không phải tham số chung"
            continue
        try:
            new[k] = P.coerce(P.PARAMS[k], v)
        except ValueError as e:
            errors[k] = str(e)
    if errors:
        raise HTTPException(400, {"errors": errors})
    S.shared = new
    results = {}
    for s in S.reg.sessions.values():
        try:
            code, res = await s.link.request("PUT", "/api/config", {"patch": patch, "persist": bool(body.get("persist"))})
            results[s.id] = res
            if code == 200:
                s.config = (await s.link.request("GET", "/api/config"))[1]
        except LinkError as e:
            results[s.id] = {"ok": False, "err": str(e)}
    return {"ok": True, "shared": S.shared, "robots": results}


# ---------------------------------------------------------------- plan / run
def _plan(body: dict) -> dict:
    grid = body.get("grid") or S.grid
    S.grid = {"cell_mm": float(grid["cell_mm"]), "cols": int(grid["cols"]), "rows": int(grid["rows"])}
    L, F = S.reg.by_role("leader"), S.reg.by_role("follower")
    cl, cf = S.reg.cfg_params(L), S.reg.cfg_params(F)
    form, ferr, fwarn = P.formation(cl, cf, S.shared)
    res = plan_path(body.get("points", []), S.grid, form, S.shared, {"rear": cf["body.axle_to_rear_mm"]})
    res["errors"] = list(ferr.values()) + res["errors"]
    res["warnings"] = fwarn + res["warnings"]
    if L is None or F is None:
        res["warnings"].insert(0, "chưa gán đủ vai trò leader/follower — đang dùng kích thước mặc định")
    res["ok"] = not res["errors"]
    res["formation"] = form
    res["geom"] = {"leader": _geom(cl), "follower": _geom(cf)}
    return res


def _geom(c: dict) -> dict:
    return {"length": c["body.length_mm"], "width": c["body.width_mm"], "front": c["body.axle_to_front_mm"],
            "rear": c["body.axle_to_rear_mm"], "tof_x": c["range.x_mm"], "tof_b": c["range.baseline_mm"],
            "track": c["wheel.track_mm"], "wheel_d": c["wheel.diameter_mm"], "r_sweep": P.sweep_radius(c)}


@app.post("/api/plan")
def plan(body: dict = Body(...)):
    return _plan(body)


@app.post("/api/run/start")
async def run_start(body: dict = Body(...)):
    if S.coord.state == "RUNNING":
        raise HTTPException(409, "đang chạy")
    res = _plan(body)
    if not res["ok"]:
        raise HTTPException(400, {"errors": res["errors"]})
    L, F = S.reg.by_role("leader"), S.reg.by_role("follower")
    if L is None or F is None:
        raise HTTPException(400, {"errors": ["cần gán 1 leader và 1 follower"]})
    path = res["path"]
    if S.sim is not None:
        st, fs = path["start"], path["follower"]["start"]
        if L.source == "SIM":
            S.sim.place(L.id, st["x"], st["y"], st["yaw_deg"])
        if F.source == "SIM":
            S.sim.place(F.id, fs["x"], fs["y"], fs["yaw_deg"])
        S.sim.collision_events = 0
        S.sim.running = True
    for s in (L, F):
        await s.link.send({"t": "cmd", "op": "reset"})
        await _req(s, "PUT", "/api/path", path)
    S.last_path = path
    S.coord.clock = S.clock
    S.coord.on_change = lambda: None
    S.coord.start(path, L, F, dict(S.shared))
    S.note(f"Bắt đầu chạy đường #{path['path_id']} ({len(path['segments'])} đoạn, {len(path['corners_s'])} góc)")
    return {"ok": True, "path": path, "warnings": res["warnings"]}


@app.post("/api/run/{action}")
async def run_action(action: str):
    if action == "pause":
        await S.coord.pause()
    elif action == "resume":
        await S.coord.resume()
    elif action == "stop":
        await S.coord.stop()
    elif action == "estop":
        for s in S.reg.sessions.values():
            try:
                await s.link.send({"t": "cmd", "op": "estop"})
            except LinkError:
                pass
        await S.coord.stop("ABORTED")
        S.note("E-STOP!", "fault")
    elif action == "reset":
        for s in S.reg.sessions.values():
            try:
                await s.link.send({"t": "cmd", "op": "reset"})
            except LinkError:
                pass
    else:
        raise HTTPException(404, "action không hợp lệ")
    return {"ok": True, "coord": S.coord.status()}


# ---------------------------------------------------------------- sim
@app.post("/api/sim/start")
async def sim_start(body: dict = Body(default={})):
    if body.get("grid"):
        g = body["grid"]
        S.grid = {"cell_mm": float(g["cell_mm"]), "cols": int(g["cols"]), "rows": int(g["rows"])}
    await S.sim_start(int(body.get("seed", 1)))
    return {"ok": True, "sim": S.sim_status()}


@app.post("/api/sim/stop")
async def sim_stop():
    await S.sim_stop()
    return {"ok": True}


@app.post("/api/sim/control")
async def sim_control(body: dict = Body(...)):
    if S.sim is None:
        raise HTTPException(400, "chưa bật mô phỏng")
    if "running" in body:
        S.sim.running = bool(body["running"])
    if "scale" in body:
        S.sim.scale = max(0.1, min(8.0, float(body["scale"])))
    if body.get("step"):
        S.sim.advance(float(body["step"]))
    return {"ok": True, "sim": S.sim_status()}


@app.post("/api/sim/physics")
async def sim_physics(body: dict = Body(...)):
    if S.sim is None:
        raise HTTPException(400, "chưa bật mô phỏng")
    rid = body.get("robot")
    if rid not in S.sim.robots:
        raise HTTPException(404, "không có robot ảo này")
    patch = {k: float(v) for k, v in body.get("mismatch", {}).items()}
    mm = S.sim.set_mismatch(rid, patch)
    S.note(f"{rid}: đổi vật lý {patch}")
    return {"ok": True, "mismatch": mm}


@app.post("/api/sim/zones")
async def sim_zones(body: dict = Body(...)):
    if S.sim is None:
        raise HTTPException(400, "chưa bật mô phỏng")
    S.sim.set_zones(body.get("zones", []))
    return {"ok": True}


@app.post("/api/sim/fault")
async def sim_fault(body: dict = Body(...)):
    if S.sim is None:
        raise HTTPException(400, "chưa bật mô phỏng")
    rid = body.pop("robot", None)
    kind = body.pop("type", None)
    if rid not in S.sim.robots:
        raise HTTPException(404, "không có robot ảo này")
    try:
        S.sim.inject(kind, rid, **body)
    except ValueError as e:
        raise HTTPException(400, str(e)) from None
    S.note(f"Gây lỗi {kind} trên {rid}", "warn")
    return {"ok": True}


# ---------------------------------------------------------------- record / replay
@app.post("/api/record/{action}")
def record(action: str):
    if action == "start":
        if S.rec_file is None:
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            S.rec_name = f"run-{datetime.now():%Y%m%d-%H%M%S}.jsonl"
            S.rec_file = open(LOG_DIR / S.rec_name, "w", encoding="utf-8")  # noqa: SIM115
    elif action == "stop":
        if S.rec_file:
            S.rec_file.close()
        S.rec_file, S.rec_name = None, None
    else:
        raise HTTPException(404)
    return {"ok": True, "recording": S.rec_name}


@app.get("/api/logs")
def logs():
    if not LOG_DIR.exists():
        return []
    return sorted((p.name for p in LOG_DIR.glob("*.jsonl")), reverse=True)


@app.post("/api/replay/start")
async def replay_start(body: dict = Body(...)):
    name = Path(str(body.get("file", ""))).name
    f = LOG_DIR / name
    if not f.exists():
        raise HTTPException(404, "không có file log")
    if S.replay_task and not S.replay_task.done():
        S.replay_task.cancel()
    S.replay_task = asyncio.create_task(S.replay(f, float(body.get("speed", 1.0))))
    return {"ok": True}


@app.post("/api/replay/stop")
async def replay_stop():
    if S.replay_task:
        S.replay_task.cancel()
    return {"ok": True}


# ---------------------------------------------------------------- ws + static
@app.websocket("/ws/ui")
async def ws_ui(ws: WebSocket):
    await ws.accept()
    S.clients.add(ws)
    try:
        await ws.send_text(json.dumps({"type": "hello", "version": __version__,
                                       "events": list(S.events)[-50:]}, ensure_ascii=False))
        while True:
            await ws.receive_text()  # UI chỉ nghe; giữ kết nối
    except WebSocketDisconnect:
        pass
    finally:
        S.clients.discard(ws)


@app.get("/")
def index():
    return FileResponse(WEB_DIR / "index.html")


app.mount("/", StaticFiles(directory=WEB_DIR), name="web")


def main() -> None:
    import argparse

    import uvicorn

    ap = argparse.ArgumentParser(description="two-robot Web Control")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO)
    uvicorn.run(app, host=a.host, port=a.port, log_level="warning")


if __name__ == "__main__":
    main()

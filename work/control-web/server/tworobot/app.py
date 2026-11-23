"""Control Server (FastAPI): REST cho UI, WebSocket /ws/ui đẩy TelemetryFrame 20 Hz.

Server không phân biệt robot ảo hay thật (chỉ dùng RobotLink). Chạy:
    tworobot-server            # http://127.0.0.1:8080
"""
from __future__ import annotations

import asyncio
import json
import math
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
from . import calibration as CAL
from . import params as P
from .coordinator import Coordinator, preflight, preflight_solo
from .discovery import local_subnet, scan
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
        self._t0 = time.monotonic()
        self._prim_waiters: dict[int, asyncio.Future] = {}
        self._prim_ids = 900000
        self.place_noise = {"pos_mm": 0.0, "yaw_deg": 0.0}  # sim: người đặt xe không chuẩn
        self.cal_results: dict[str, dict] = {}              # kết quả hiệu chuẩn gần nhất theo robot+loại

    async def run_prim(self, sess: RobotSession, prim: str, timeout: float = 60.0, **args) -> dict:
        """Chạy 1 primitive trên robot (ngoài coordinator), trả bản tin prim_done."""
        self._prim_ids += 1
        pid = self._prim_ids
        fut = asyncio.get_running_loop().create_future()
        self._prim_waiters[pid] = fut
        seq = pid
        ack = asyncio.get_running_loop().create_future()
        self.coord._acks[seq] = ack
        try:
            await sess.link.send({"t": "cmd", "op": "prim", "seq": seq, "id": pid, "prim": prim, **args})
        except LinkError as e:
            self._prim_waiters.pop(pid, None)
            raise HTTPException(502, str(e)) from None
        if ack.done() and ack.result().get("t") == "nack":
            self._prim_waiters.pop(pid, None)
            return {"ok": False, "err": ack.result().get("err"), "result": {}}
        self.coord._acks.pop(seq, None)
        try:
            return await asyncio.wait_for(fut, timeout / max(self.sim.scale if self.sim else 1.0, 0.25) + 5)
        except asyncio.TimeoutError:
            self._prim_waiters.pop(pid, None)
            return {"ok": False, "err": "TIMEOUT", "result": {}}

    def place_at_start(self, L: RobotSession, F: RobotSession | None, path: dict) -> None:
        """Robot ảo: đặt vào vị trí xuất phát (kèm sai lệch ngẫu nhiên place_noise như người đặt)."""
        import random
        if self.sim is None:
            return
        pn, yn = self.place_noise["pos_mm"], self.place_noise["yaw_deg"]
        for sess, pose, lateral in ((L, path["start"], 0.0), (F, path["follower"]["start"], 1.0)):
            if sess is None or sess.source != "SIM":
                continue
            th = math.radians(pose["yaw_deg"])
            dx = random.uniform(-pn, pn)
            dy = random.uniform(-pn, pn) * lateral
            self.sim.place(sess.id, pose["x"] + dx * math.cos(th) - dy * math.sin(th),
                           pose["y"] + dx * math.sin(th) + dy * math.cos(th),
                           pose["yaw_deg"] + random.uniform(-yn, yn))
        self.sim.collision_events = 0

    # ------------------------------------------------------------ time
    def clock(self) -> float:
        if self.sim is not None and all(s.source == "SIM" for s in self.reg.sessions.values()):
            return self.sim.t
        return time.monotonic() - self._t0  # robot thật: giây kể từ lúc server khởi động

    # ------------------------------------------------------------ events
    def _on_event(self, sess: RobotSession, msg: dict) -> None:
        if msg.get("e") == "prim_done":
            fut = self._prim_waiters.pop(msg.get("id"), None)
            if fut is not None and not fut.done():
                fut.set_result(msg)
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
                "seed": self.sim.seed, "collisions": self.sim.collision_events, "place_noise": self.place_noise,
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

    # ------------------------------------------------------------ real robots (HTTP)
    async def add_real(self, base: str, info: dict) -> RobotSession:
        rid = str(info["id"])
        old = self.reg.sessions.get(rid)
        if old is not None:
            if old.source == "REAL" and old.ip == base:
                return old
            self.reg.remove(rid)
        link = NetLink(base)
        s = RobotSession(rid, info.get("name", rid), link, "REAL", role=info.get("role"), ip=base)
        s.info = info
        s.last_seen = time.monotonic()
        self.reg.add(s)
        link.start()
        try:
            # Tham số chung 2 xe: PC là bản chuẩn → đẩy xuống RAM của robot khi kết nối.
            await link.request("PUT", "/api/config", {"patch": self.shared})
            await self.reg.refresh(s)
        except LinkError as e:
            self.note(f"{s.name}: lỗi khi đọc cấu hình ({e})", "warn")
        self.note(f"Kết nối robot thật {s.name} @ {base}")
        return s

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
    S._bg = [asyncio.create_task(S.broadcaster())]
    yield
    for t in S._bg:
        t.cancel()
    await S.sim_stop()
    for rid in list(S.reg.sessions):
        S.reg.remove(rid)


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


@app.get("/api/discover/default")
def discover_default():
    return {"subnet": local_subnet()}


@app.post("/api/discover")
async def discover(body: dict = Body(...)):
    """Quét HTTP GET /api/info trên các mục tiêu (subnet, IP, IP:cổng, IP:cổng1-cổng2)."""
    targets = body.get("targets") or [local_subnet()]
    try:
        found = await scan(list(targets))
    except ValueError as e:
        raise HTTPException(400, str(e)) from None
    added = []
    for base, info in found:
        s = await S.add_real(base, info)
        added.append({"id": s.id, "name": s.name, "base": base, "fw": info.get("fw")})
    S.note(f"Quét {', '.join(targets)}: thấy {len(found)} robot")
    return {"ok": True, "found": added}


@app.delete("/api/robots/{rid}")
async def remove_robot(rid: str):
    s = _sess(rid)
    if s.source != "REAL":
        raise HTTPException(400, "robot ảo: tắt bằng nút Tắt mô phỏng")
    if S.coord.state == "RUNNING" and s in (S.coord.L, S.coord.F):
        raise HTTPException(409, "robot đang chạy")
    S.reg.remove(rid)
    S.note(f"Đã ngắt {s.name}")
    return {"ok": True}


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


def manual_jog_motion(body: dict) -> tuple[float, float]:
    try:
        v = float(body.get("v", 0.0))
        w = float(body.get("w", 0.0))
    except (TypeError, ValueError):
        raise HTTPException(400, "v/w phải là số") from None
    if not math.isfinite(v) or not math.isfinite(w):
        raise HTTPException(400, "v/w phải hữu hạn")
    return max(-150.0, min(150.0, v)), max(-2.0, min(2.0, w))


@app.post("/api/robots/{rid}/jog")
async def jog(rid: str, body: dict = Body(...)):
    s = _sess(rid)
    v, w = manual_jog_motion(body)
    try:
        await s.link.send({"t": "cmd", "op": "jog", "v": v, "w": w})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    return {"ok": True}


@app.post("/api/robots/{rid}/jog/stop")
async def jog_stop(rid: str):
    s = _sess(rid)
    try:
        await s.link.send({"t": "cmd", "op": "stop"})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    return {"ok": True}


@app.post("/api/robots/{rid}/calib_imu")
async def calib_imu(rid: str):
    """Calib bias gyro: robot phải đứng yên imu.calib_time_s giây. Kết quả về qua sự kiện calib_done."""
    s = _sess(rid)
    try:
        await s.link.send({"t": "cmd", "op": "calib_imu"})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    return {"ok": True}


@app.post("/api/robots/{rid}/calib_tof")
async def calib_tof(rid: str, body: dict = Body(default={})):
    """Hiệu chuẩn 2 mắt ToF bằng cách quay ±40° trước một mặt phẳng rộng (wiki/algorithms.md §6.1).

    Robot thật: người dùng đặt xe trước tường/hộp phẳng ≥ 40 cm, cách ~150–300 mm, gần vuông góc.
    Robot ảo: server tự đặt xe trước tường bản đồ (lệch ngẫu nhiên vài độ) như người đặt.
    Trả kết quả; chưa ghi config — UI hỏi người dùng rồi mới Apply + Save.
    """
    s = _sess(rid)
    if S.coord.state == "RUNNING":
        raise HTTPException(409, "đang chạy đường")
    if s.source == "SIM" and S.sim is not None and body.get("place", True):
        import random as _r
        w = S.sim.world
        r = S.sim.robots[rid]
        S.sim.place(rid, w.width + w.wall_margin - r.body.p.front - float(body.get("dist_mm", 200)),
                    w.height / 2, _r.uniform(-6, 6))
        await asyncio.sleep(0.8 / max(S.sim.scale, 0.1))  # để xe đứng yên ổn định
    S._prim_ids += 1
    pid = S._prim_ids
    fut = asyncio.get_running_loop().create_future()
    S._prim_waiters[pid] = fut
    args = {k: body[k] for k in ("scan_deg", "rate_dps", "wall_mm") if body.get(k) is not None}
    try:
        await s.link.send({"t": "cmd", "op": "prim", "id": pid, "prim": "TOF_CAL", **args})
    except LinkError as e:
        raise HTTPException(502, str(e)) from None
    S.note(f"{s.name}: hiệu chuẩn ToF (quét ±40°)…")
    try:
        msg = await asyncio.wait_for(fut, timeout=90)
    except asyncio.TimeoutError:
        S._prim_waiters.pop(pid, None)
        raise HTTPException(504, "hết thời gian chờ hiệu chuẩn ToF (robot bị từ chối lệnh? xem tab Sự kiện)") from None
    if not msg.get("ok"):
        hint = {"PERP_OUT_OF_SCAN": "không thấy hướng vuông góc trong ±40°: đặt xe gần vuông góc hơn, mặt phẳng rộng hơn",
                "TOO_FEW_SAMPLES": "ít số đo hợp lệ: mặt phẳng quá xa/gần hoặc quá hẹp",
                "imu_uncal": "gyro chưa calib"}.get(msg.get("err"), "")
        raise HTTPException(400, f"hiệu chuẩn ToF lỗi: {msg.get('err')}. {hint}")
    res = msg["result"]
    S.note(f"{s.name}: ToF lệch 2 mắt {res['c_mm']:+.1f} mm (Δψ lệch {res['dpsi_bias_deg']:+.1f}°), "
           f"baseline hiệu dụng {res['baseline_mm']:.0f} mm")
    return {"ok": True, "result": res}


@app.post("/api/robots/{rid}/save_temp_coeff")
async def save_temp_coeff(rid: str):
    """Lưu hệ số bias-nhiệt độ robot đã học vào config + flash (dùng ngay từ lần chạy sau)."""
    s = _sess(rid)
    imu = (s.last_tel or {}).get("imu") or {}
    if not imu.get("k_learned"):
        raise HTTPException(400, "robot chưa học được hệ số (cần chênh nhiệt ≥ 0.5 °C giữa các lần đứng yên)")
    k = round(float(imu["k_dps_c"]), 4)
    res = await _req(s, "PUT", "/api/config", {"patch": {"imu.temp_coeff_dps_c": k}, "persist": True})
    s.config = await _req(s, "GET", "/api/config")
    S.note(f"{s.name}: lưu hệ số nhiệt gyro {k} °/s/°C")
    return {"ok": True, "k": k, **res}


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
def _solo(body: dict) -> bool:
    """mode: "solo" (1 xe = leader chạy một mình theo đường vẽ) | "pair" (leader + follower) |
    "follow" (1 xe bám tấm cầm tay — không dùng đường; bản đồ kiểm tra như 1 xe)."""
    return body.get("mode") in ("solo", "follow")


def _pick_robot(body: dict, required: bool = True) -> RobotSession | None:
    """Chế độ 1 xe / bám tấm: xe do người dùng chọn (`rid`), KHÔNG dựa vào vai trò.
    Không gửi rid mà chỉ có 1 xe đang kết nối → dùng xe đó."""
    rid = body.get("rid")
    if rid:
        return _sess(rid)
    if len(S.reg.sessions) == 1:
        return next(iter(S.reg.sessions.values()))
    if required:
        raise HTTPException(400, {"errors": ["chọn xe chạy (ô chọn xe ở tab Bản đồ / Chạy)"]})
    return None


def _plan(body: dict) -> dict:
    grid = body.get("grid") or S.grid
    S.grid = {"cell_mm": float(grid["cell_mm"]), "cols": int(grid["cols"]), "rows": int(grid["rows"])}
    solo = _solo(body)
    if solo:  # 1 xe: kích thước lấy từ xe được chọn, không phụ thuộc vai trò
        L, F = _pick_robot(body, required=False), None
    else:
        L, F = S.reg.by_role("leader"), S.reg.by_role("follower")
    cl, cf = S.reg.cfg_params(L), S.reg.cfg_params(F)
    form, ferr, fwarn = P.formation(cl, cf, S.shared)
    res = plan_path(body.get("points", []), S.grid, form, S.shared, {"rear": cf["body.axle_to_rear_mm"]}, solo=solo)
    if solo:  # ràng buộc đội hình 2 xe không áp dụng
        ferr, fwarn = {}, []
    res["errors"] = list(ferr.values()) + res["errors"]
    res["warnings"] = fwarn + res["warnings"]
    if solo and L is None:
        res["warnings"].insert(0, "chế độ 1 xe: chưa chọn xe — đang dùng kích thước mặc định")
    elif not solo and (L is None or F is None):
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


def _clear_stale_coord() -> None:
    """Lần thử mới: bỏ kết quả kiểm tra / lỗi của lần trước để UI không hiện số cũ."""
    if S.coord.state != "RUNNING":
        S.coord.checks, S.coord.error = [], None
        if S.coord.state == "FAULT":
            S.coord.state = "IDLE"


def _run_robots(body: dict) -> tuple[RobotSession, RobotSession | None]:
    """Xe tham gia: chế độ 1 xe → (leader, None); 2 xe → (leader, follower)."""
    if _solo(body):  # 1 xe: xe được chọn, vai trò không liên quan
        return _pick_robot(body), None
    L, F = S.reg.by_role("leader"), S.reg.by_role("follower")
    if L is None or F is None:
        raise HTTPException(400, {"errors": ["cần gán 1 leader và 1 follower (hoặc chọn chế độ 1 xe)"]})
    return L, F


@app.post("/api/run/start")
async def run_start(body: dict = Body(...)):
    if S.coord.state == "RUNNING":
        raise HTTPException(409, "đang chạy")
    _clear_stale_coord()
    res = _plan(body)
    if not res["ok"]:
        raise HTTPException(400, {"errors": res["errors"]})
    L, F = _run_robots(body)
    path = res["path"]
    if S.sim is not None:
        if body.get("place", True):
            S.place_at_start(L, F, path)
        S.sim.running = True
    for s in [x for x in (L, F) if x is not None]:
        await s.link.send({"t": "cmd", "op": "reset"})
        await _req(s, "PUT", "/api/path", path)
    S.last_path = path
    S.coord.clock = S.clock
    S.coord.on_change = lambda: None
    S.coord.start(path, L, F, dict(S.shared))
    S.note(f"Bắt đầu chạy đường #{path['path_id']} ({len(path['segments'])} đoạn, {len(path['corners_s'])} góc)"
           + (f" · 1 xe ({L.name})" if F is None else ""))
    return {"ok": True, "path": path, "warnings": res["warnings"]}


@app.post("/api/run/check")
async def run_check(body: dict = Body(...)):
    """Chạy thử bước 'Kiểm tra trước khi chạy' (không chạy đường). Robot ảo: đặt vào vị trí xuất phát trước."""
    if S.coord.state == "RUNNING":
        raise HTTPException(409, "đang chạy")
    _clear_stale_coord()
    res = _plan(body)
    if not res["ok"]:
        raise HTTPException(400, {"errors": res["errors"]})
    L, F = _run_robots(body)
    if S.sim is not None and body.get("place", True):
        S.place_at_start(L, F, res["path"])
        await asyncio.sleep(0.6 / max(S.sim.scale, 0.1))
    for s in [x for x in (L, F) if x is not None]:  # cần biết vị trí leader rear (formation) để tính gap
        await s.link.send({"t": "cmd", "op": "reset"})
        await _req(s, "PUT", "/api/path", res["path"])
    await asyncio.sleep(0.15)
    checks = (await preflight_solo(L, S.shared) if F is None
              else await preflight(L, F, res["formation"], S.shared, S.run_prim))
    S.last_path = res["path"]
    ok = all(c["ok"] for c in checks)
    S.coord.checks = checks
    S.note("Kiểm tra trước khi chạy: " + ("ĐẠT" if ok else "KHÔNG ĐẠT"), "info" if ok else "warn")
    return {"ok": ok, "checks": checks}


# ---------------------------------------------------------------- calibration wizards
CAL_PRIM = {"drive": "CAL_DRIVE", "turn": "CAL_TURN", "gap": "SNAPSHOT", "ks": "CAL_KS"}


def _truth(s: RobotSession) -> dict | None:
    return (s.last_tel or {}).get("truth")


# ---- Encoder quay tay: bắt đầu (ghi mốc tick) → người dùng quay đúng N vòng tới → kết thúc (tính) → lưu
ENC_CAL: dict[str, dict] = {}


def _enc_now(s: RobotSession, side: str) -> int:
    enc = (s.last_tel or {}).get("enc")
    if not enc:
        raise HTTPException(400, "robot chưa gửi tick encoder (cần firmware ≥ 0.2.1)")
    return int(enc[side])


@app.post("/api/robots/{rid}/cal_enc/{action}")
async def cal_enc(rid: str, action: str, body: dict = Body(default={})):
    s = _sess(rid)
    side = body.get("side")
    if side not in ("l", "r"):
        raise HTTPException(400, "side phải là l | r")
    key = f"{rid}:{side}"
    if action == "start":
        ENC_CAL[key] = {"t0": _enc_now(s, side)}
        return {"ok": True, "start_ticks": ENC_CAL[key]["t0"]}
    if action == "finish":
        if key not in ENC_CAL:
            raise HTTPException(400, "chưa bấm Bắt đầu cho bánh này")
        delta = _enc_now(s, side) - ENC_CAL[key]["t0"]
        cfg = (await _req(s, "GET", "/api/config"))["params"]
        other = ENC_CAL.get(f"{rid}:{'r' if side == 'l' else 'l'}", {}).get("info")
        try:
            patch, info = CAL.encoder(cfg, side, delta, float(body.get("revs", 2)), other)
        except CAL.CalError as e:
            raise HTTPException(400, str(e)) from None
        ENC_CAL[key].update(info=info, patch=patch)
        return {"ok": True, "patch": patch, "info": info}
    if action == "save":
        st = ENC_CAL.get(key, {})
        if "patch" not in st:
            raise HTTPException(400, "chưa có kết quả để lưu")
        r = await _req(s, "PUT", "/api/config", {"patch": st["patch"], "persist": True})
        s.config = await _req(s, "GET", "/api/config")
        S.note(f"{s.name}: hiệu chuẩn encoder {side} → {st['patch']} (đã lưu flash)")
        return {"ok": True, "patch": st["patch"], "cfg_rev": r.get("cfg_rev")}
    raise HTTPException(404, "action: start | finish | save")


@app.post("/api/robots/{rid}/cal/{kind}")
async def cal_run(rid: str, kind: str, body: dict = Body(default={})):
    """Bước 1 hiệu chuẩn: robot thực hiện (đi thẳng / quay / đo khoảng cách) và báo số đo của nó.
    Robot ảo: trả thêm 'sim_measured' — số đo thật trong mô phỏng (thay cho thước của người dùng)."""
    if kind not in CAL_PRIM:
        raise HTTPException(404, "loại hiệu chuẩn: drive | turn | gap | ks")
    s = _sess(rid)
    if S.coord.state == "RUNNING":
        raise HTTPException(409, "đang chạy đường")
    await s.link.send({"t": "cmd", "op": "reset"})
    args: dict = {}
    if kind == "drive":
        args = {"mm": float(body.get("mm", 1000))}
    elif kind == "turn":
        args = {"deg": float(body.get("deg", 360))}
    elif kind == "gap":
        args = {"n": int(body.get("n", 30))}
        if s.source == "SIM" and S.sim is not None and body.get("place", True):
            # Robot ảo: đặt xe này sau robot kia đúng known_mm (mặt mắt ToF → mặt đuôi), thẳng hàng.
            other = next((o for o in S.reg.sessions.values() if o.source == "SIM" and o.id != rid), None)
            if other is None:
                raise HTTPException(400, "cần robot ảo thứ 2 làm 'xe trước'")
            me, ot = S.sim.robots[rid].body, S.sim.robots[other.id].body
            gap_c = float(body.get("known_mm", 100)) + me.p.tof_x + ot.p.rear
            S.sim.place(other.id, ot.x, ot.y, math.degrees(ot.th))
            S.sim.place(rid, ot.x - gap_c * math.cos(ot.th), ot.y - gap_c * math.sin(ot.th), math.degrees(ot.th))
            await asyncio.sleep(0.6 / max(S.sim.scale, 0.1))
    t0 = _truth(s)
    msg = await S.run_prim(s, CAL_PRIM[kind], timeout=120, **args)
    if not msg.get("ok"):
        raise HTTPException(400, f"{s.name}: {CAL_PRIM[kind]} lỗi {msg.get('err')}")
    await asyncio.sleep(0.3)
    t1 = _truth(s)
    sim_measured = None
    if t0 and t1:
        if kind == "drive":
            sim_measured = round(math.hypot(t1["x"] - t0["x"], t1["y"] - t0["y"]), 1)
        elif kind == "turn" and "yaw_acc" in t1:
            sim_measured = round(math.degrees(t1["yaw_acc"] - t0["yaw_acc"]), 2)
        elif kind == "gap" and S.sim is not None:
            other = next((o for o in S.reg.sessions.values() if o.id != rid and o.source == "SIM"), None)
            if other:
                me, ot = S.sim.robots[rid].body, S.sim.robots[other.id].body
                sim_measured = round(math.hypot(ot.x - me.x, ot.y - me.y) - me.p.tof_x - ot.p.rear, 1)
    S.cal_results[f"{rid}:{kind}"] = msg["result"]
    return {"ok": True, "kind": kind, "result": msg["result"], "sim_measured": sim_measured}


@app.post("/api/robots/{rid}/cal/{kind}/apply")
async def cal_apply(rid: str, kind: str, body: dict = Body(...)):
    """Bước 2: người dùng nhập số đo thực tế → tính hệ số mới → ghi config + lưu flash."""
    s = _sess(rid)
    res = S.cal_results.get(f"{rid}:{kind}")
    if res is None:
        raise HTTPException(400, "chưa chạy bước 1 cho robot này")
    cfg = (await _req(s, "GET", "/api/config"))["params"]
    try:
        if kind == "drive":
            patch, info = CAL.drive(cfg, res, float(body["measured"]))
        elif kind == "turn":
            patch, info = CAL.turn(cfg, res, float(body["measured"]))
        elif kind == "gap":
            patch, info = CAL.gap(cfg, res, float(body["measured"]), bool(body.get("keep_diff", True)))
        elif kind == "ks":  # không cần người đo: tính thẳng từ kết quả robot
            patch, info = CAL.ks(cfg, res)
        else:
            raise HTTPException(404, "loại hiệu chuẩn: drive | turn | gap | ks")
    except (CAL.CalError, KeyError, ValueError) as e:
        raise HTTPException(400, str(e)) from None
    if body.get("dry_run"):
        return {"ok": True, "patch": patch, "info": info, "saved": False}
    r = await _req(s, "PUT", "/api/config", {"patch": patch, "persist": True})
    s.config = await _req(s, "GET", "/api/config")
    S.note(f"{s.name}: hiệu chuẩn {kind} → {patch} (đã lưu flash)")
    return {"ok": True, "patch": patch, "info": info, "saved": True, "cfg_rev": r.get("cfg_rev")}


# ---------------------------------------------------------------- 1 xe bám tấm phẳng cầm tay
FOLLOW_PLATE: dict = {}  # rid → {"d_target", "v_max"}


def _plate_robot(body: dict) -> RobotSession:
    """Xe bám tấm: xe do người dùng chọn (rid) — vai trò không liên quan."""
    return _pick_robot(body)


@app.post("/api/follow/start")
async def follow_start(body: dict = Body(default={})):
    """Chạy FOLLOW_PLATE từ IDLE (không cần path/ARM): giữ mặt kính ToF → tấm = d_target mm. Dừng bằng /api/follow/stop."""
    if S.coord.state == "RUNNING":
        raise HTTPException(409, "đang chạy đường")
    s = _plate_robot(body)
    # Mặc định dùng cấu hình robot (tab Cấu hình → nhóm "Bám tấm"); body có thể ghi đè riêng lần chạy này
    args = {k: float(body[k]) for k in ("d_target", "v_max", "v_back", "hold_band") if body.get(k) is not None}
    cfg = ((s.config or {}).get("params") or {})
    d = args.get("d_target", cfg.get("plate.dist_mm", 200.0))
    await s.link.send({"t": "cmd", "op": "reset"})
    if not ((s.last_tel or {}).get("imu") or {}).get("ok"):
        raise HTTPException(400, {"errors": [f"{s.name}: IMU chưa calib — để xe đứng yên vài giây rồi thử lại"]})
    if S.sim is not None:
        S.sim.running = True
    res = await _req(s, "POST", "/api/cmd", {"op": "prim", "id": 800000 + int(time.time()) % 100000,
                                            "prim": "FOLLOW_PLATE", **args})
    if not res.get("ok", True):
        raise HTTPException(400, {"errors": [f"{s.name} từ chối: {res.get('err')}"]})
    FOLLOW_PLATE[s.id] = args
    S.note(f"{s.name}: bám tấm phẳng, giữ {d:.0f} mm (mặt kính ToF → tấm)")
    return {"ok": True, "rid": s.id, "d_target": d, **args}


@app.post("/api/follow/stop")
async def follow_stop(body: dict = Body(default={})):
    ids = [body["rid"]] if body.get("rid") else list(FOLLOW_PLATE) or list(S.reg.sessions)
    for rid in ids:
        if rid in S.reg.sessions:
            try:
                await S.reg.sessions[rid].link.send({"t": "cmd", "op": "stop"})
            except LinkError:
                pass
        FOLLOW_PLATE.pop(rid, None)
    S.note("Dừng bám tấm")
    return {"ok": True}


@app.post("/api/run/{action}")
async def run_action(action: str):
    if action == "pause":
        await S.coord.pause()
    elif action == "resume":
        await S.coord.resume()
    elif action == "stop":
        await S.coord.stop()
        if FOLLOW_PLATE:  # đang bám tấm (không qua điều phối)
            await follow_stop({})
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
    if "place_noise_mm" in body:
        S.place_noise["pos_mm"] = max(0.0, min(100.0, float(body["place_noise_mm"])))
    if "place_noise_deg" in body:
        S.place_noise["yaw_deg"] = max(0.0, min(20.0, float(body["place_noise_deg"])))
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

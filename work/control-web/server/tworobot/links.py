"""Kết nối PC ⇄ robot. Server chỉ làm việc với RobotLink, không biết sim hay thật.

- SimLink: robot ảo trong cùng process (gọi hàm), message đúng định dạng giao thức.
- NetLink: robot thật (ESP32) — CHỈ HTTP qua LAN (wiki/protocol.md, quyết định D25):
    * lệnh       : POST /api/cmd            → ack/nack trả ngay trong response
    * trạng thái : GET  /api/state?since=N  (hỏi 20 Hz) → telemetry + sự kiện mới (eseq > N)
    * relay      : POST /api/relay          (trạng thái leader → follower, qua PC)
    * heartbeat  : chính các lần hỏi /api/state; robot mất request > link_timeout_ms → phanh
"""
from __future__ import annotations

import asyncio
import time
from typing import TYPE_CHECKING, Callable

import httpx

if TYPE_CHECKING:
    from .sim.engine import SimEngine, SimRobot


class LinkError(Exception):
    pass


class RobotLink:
    source = "?"

    def __init__(self):
        self.on_message: Callable[[dict], None] = lambda msg: None

    async def request(self, method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
        raise NotImplementedError

    async def send(self, msg: dict) -> None:
        raise NotImplementedError

    def post(self, msg: dict) -> None:
        """Gửi không chờ (relay)."""
        raise NotImplementedError

    def stats(self) -> dict:
        return {}

    def close(self) -> None:
        pass


class SimLink(RobotLink):
    source = "SIM"

    def __init__(self, engine: "SimEngine", robot: "SimRobot"):
        super().__init__()
        self.engine = engine
        self.robot = robot
        robot.fw.emit = self._from_fw
        robot.fw.link_watch = True  # engine bơm heartbeat theo thời gian sim

    def _from_fw(self, msg: dict) -> None:
        if not self.robot.link_alive and msg.get("t") == "tel":
            return  # mất link: PC không nhận được telemetry
        if msg.get("t") == "tel":
            msg["truth"] = self.engine.truth(self.robot.id)
            msg["sim_t"] = self.engine.t
        self.on_message(msg)

    async def request(self, method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
        if not self.robot.link_alive:
            raise LinkError("mất kết nối")
        return self.robot.fw.handle_request(method, path, body)

    async def send(self, msg: dict) -> None:
        self.post(msg)

    def post(self, msg: dict) -> None:
        if self.robot.link_alive:
            self.robot.fw.handle_ws(msg)

    def close(self) -> None:
        self.robot.fw.emit = lambda msg: None


class NetLink(RobotLink):
    """Robot thật qua HTTP. base: 'http://192.168.1.42' (hoặc kèm cổng)."""
    source = "REAL"

    def __init__(self, base: str, poll_hz: float = 20.0):
        super().__init__()
        self.base = base.rstrip("/")
        self.period = 1.0 / poll_hz
        self.client = httpx.AsyncClient(timeout=httpx.Timeout(1.0, connect=0.5),
                                        limits=httpx.Limits(max_keepalive_connections=4))
        self.eseq = 0
        self.online = False
        self.errors = 0
        self.rtt_ms: float | None = None
        self.events_lost = 0
        self._task: asyncio.Task | None = None
        self._relay_busy = False

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.get_running_loop().create_task(self._poll())

    async def _poll(self) -> None:
        while True:
            t0 = time.perf_counter()
            try:
                r = await self.client.get(f"{self.base}/api/state", params={"since": self.eseq}, timeout=0.5)
                d = r.json()
                self.rtt_ms = (time.perf_counter() - t0) * 1000
                self.online = True
                if d.get("eseq", 0) < self.eseq:  # robot khởi động lại → đếm lại
                    self.eseq = 0
                if d.get("events_lost"):
                    self.events_lost += 1
                for e in d.get("events", []):
                    self.eseq = max(self.eseq, int(e.get("eseq", 0)))
                    self.on_message(e)
                tel = d.get("tel")
                if tel:
                    tel["link"] = {"rtt_ms": round(self.rtt_ms, 1), "errors": self.errors}
                    self.on_message(tel)
            except (httpx.HTTPError, ValueError):
                # Một gói WiFi bị kẹt: hỏi lại ngay — mỗi lần hỏi là heartbeat, chờ thêm chu kỳ dễ chạm LINK_LOST
                self.online = False
                self.errors += 1
                await asyncio.sleep(0.01)
                continue
            await asyncio.sleep(max(0.0, self.period - (time.perf_counter() - t0)))

    async def request(self, method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
        try:
            r = await self.client.request(method, self.base + path,
                                          json=body if body is not None and method != "GET" else None,
                                          params=body if method == "GET" and body else None)
        except httpx.HTTPError as e:
            raise LinkError(f"{self.base}: {type(e).__name__}") from None
        try:
            data = r.json()
        except ValueError:
            data = {"ok": False, "err": r.text[:200]}
        return r.status_code, data

    async def send(self, msg: dict) -> None:
        t = msg.get("t")
        if t == "cmd":
            body = {k: v for k, v in msg.items() if k not in ("t", "seq")}
            _, res = await self.request("POST", "/api/cmd", body)
            self.on_message({"t": "ack" if res.get("ok") else "nack", "seq": msg.get("seq"), "err": res.get("err")})
        elif t == "relay":
            await self.request("POST", "/api/relay", {k: v for k, v in msg.items() if k != "t"})
        elif t == "cfg_set":
            await self.request("PUT", "/api/config", {"patch": {msg["key"]: msg["value"]}})
        # "hb": không cần — mỗi lần hỏi /api/state đã là heartbeat

    def post(self, msg: dict) -> None:
        if self._relay_busy:  # chỉ giữ 1 relay đang bay: bỏ gói cũ, gói sau mới hơn
            return
        self._relay_busy = True

        async def go():
            try:
                await self.send(msg)
            except LinkError:
                pass
            finally:
                self._relay_busy = False
        asyncio.get_running_loop().create_task(go())

    def stats(self) -> dict:
        return {"online": self.online, "rtt_ms": self.rtt_ms, "errors": self.errors, "base": self.base}

    def close(self) -> None:
        if self._task:
            self._task.cancel()
        try:
            asyncio.get_running_loop().create_task(self.client.aclose())
        except RuntimeError:
            pass

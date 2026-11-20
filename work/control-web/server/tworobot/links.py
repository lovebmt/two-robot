"""Kết nối PC ⇄ robot. Server chỉ làm việc với RobotLink, không biết sim hay thật.

- SimLink: robot ảo trong cùng process. Trao đổi đúng các message JSON của giao thức
  (wiki/protocol.md), chỉ khác lớp vận chuyển (gọi hàm thay vì HTTP/WS/UDP).
- NetLink: ESP32 thật qua HTTP + WebSocket + UDP — làm ở M4 khi có firmware.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Callable

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
        """Gửi không chờ (dùng cho relay/heartbeat tần số cao)."""
        raise NotImplementedError

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
    """ESP32 thật. Chưa cài đặt: phụ thuộc firmware (roadmap M4)."""
    source = "REAL"

    def __init__(self, ip: str):
        super().__init__()
        self.ip = ip

    async def request(self, method, path, body=None):
        raise LinkError("NetLink chưa được cài đặt (chờ firmware ESP32 — roadmap M4)")

    async def send(self, msg):
        raise LinkError("NetLink chưa được cài đặt (chờ firmware ESP32 — roadmap M4)")

    def post(self, msg):
        pass

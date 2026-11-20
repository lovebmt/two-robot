"""Danh sách robot đang biết (sim hoặc thật), vai trò, cache config, telemetry mới nhất, relay."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

from . import params as P
from .links import LinkError, RobotLink


@dataclass
class RobotSession:
    id: str
    name: str
    link: RobotLink
    source: str
    role: str | None = None
    last_tel: dict | None = None
    last_seen: float = 0.0
    config: dict | None = None  # {"params":..., "derived":..., "cfg_rev":..., ...}
    info: dict = field(default_factory=dict)
    ip: str | None = None

    def summary(self) -> dict:
        tel = self.last_tel or {}
        return {
            "id": self.id, "name": self.name, "source": self.source, "role": self.role,
            "state": tel.get("state", "?"), "cfg_rev": tel.get("cfg_rev"),
            "online": (time.monotonic() - self.last_seen) < 2.0, "ip": self.ip,
            "flags": tel.get("flags", []), "fault": tel.get("fault"),
            "fw": self.info.get("fw"),
        }


class Registry:
    def __init__(self):
        self.sessions: dict[str, RobotSession] = {}
        self.on_event: Callable[[RobotSession, dict], None] = lambda s, m: None
        self.on_tel: Callable[[RobotSession, dict], None] = lambda s, m: None

    def add(self, sess: RobotSession) -> None:
        self.sessions[sess.id] = sess
        sess.link.on_message = lambda msg, s=sess: self._on_message(s, msg)

    def remove(self, rid: str) -> None:
        s = self.sessions.pop(rid, None)
        if s:
            s.link.close()

    def get(self, rid: str) -> RobotSession:
        if rid not in self.sessions:
            raise KeyError(rid)
        return self.sessions[rid]

    def by_role(self, role: str) -> RobotSession | None:
        for s in self.sessions.values():
            if s.role == role:
                return s
        return None

    def _on_message(self, sess: RobotSession, msg: dict) -> None:
        sess.last_seen = time.monotonic()
        if msg.get("t") == "tel":
            sess.last_tel = msg
            if sess.role == "leader":  # relay trạng thái leader → follower (qua PC)
                f = self.by_role("follower")
                if f is not None:
                    est = msg.get("est", {})
                    f.link.post({"t": "relay", "t_ms": msg.get("t_ms"), "s": est.get("s", 0.0),
                                 "v": est.get("v", 0.0), "phase": msg.get("state")})
            self.on_tel(sess, msg)
        else:
            self.on_event(sess, msg)

    async def refresh(self, sess: RobotSession) -> None:
        code, info = await sess.link.request("GET", "/api/info")
        if code == 200:
            sess.info = info
            sess.role = info.get("role")
        code, cfg = await sess.link.request("GET", "/api/config")
        if code == 200:
            sess.config = cfg
        sess.last_seen = time.monotonic()

    async def set_role(self, rid: str, role: str | None) -> None:
        sess = self.get(rid)
        if role in ("leader", "follower"):
            other = self.by_role(role)
            if other is not None and other.id != rid:
                code, res = await other.link.request("PUT", "/api/role", {"role": None})
                if code != 200:
                    raise LinkError(f"{other.id}: {res.get('err')}")
                other.role = None
        code, res = await sess.link.request("PUT", "/api/role", {"role": role})
        if code != 200:
            raise LinkError(res.get("err", "lỗi"))
        sess.role = role

    def cfg_params(self, sess: RobotSession | None) -> dict:
        if sess is None or not sess.config:
            return P.defaults()
        return sess.config["params"]

"""Dò robot thật trong LAN qua UDP beacon (wiki/protocol.md §1)."""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Callable

BEACON_PORT = 47800
log = logging.getLogger("tworobot.discovery")


class _BeaconProtocol(asyncio.DatagramProtocol):
    def __init__(self, on_beacon: Callable[[dict, str], None]):
        self.on_beacon = on_beacon

    def datagram_received(self, data: bytes, addr) -> None:
        try:
            msg = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if isinstance(msg, dict) and msg.get("t") == "beacon" and msg.get("id"):
            self.on_beacon(msg, addr[0])


async def start_listener(on_beacon: Callable[[dict, str], None], port: int = BEACON_PORT):
    loop = asyncio.get_running_loop()
    try:
        transport, _ = await loop.create_datagram_endpoint(
            lambda: _BeaconProtocol(on_beacon), local_addr=("0.0.0.0", port), allow_broadcast=True)
        log.info("đang nghe beacon UDP :%d", port)
        return transport
    except OSError as e:
        log.warning("không mở được UDP :%d (%s) — bỏ qua dò robot thật", port, e)
        return None

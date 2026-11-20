"""Giao tiếp robot thật qua HTTP (D25): robot HTTP ảo ⇄ NetLink ⇄ coordinator, qua socket thật."""
import asyncio
import socket
import time

import httpx
import pytest

from tworobot import params as P
from tworobot.coordinator import Coordinator
from tworobot.discovery import expand, scan
from tworobot.links import NetLink
from tworobot.planner import plan_path
from tworobot.registry import Registry, RobotSession
from tworobot.sim.vrobot import VRobotFarm

GRID = {"cell_mm": 100, "cols": 30, "rows": 18}


def _free_port_pair() -> int:
    for _ in range(50):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            p = s.getsockname()[1]
        try:
            with socket.socket() as s2:
                s2.bind(("127.0.0.1", p + 1))
            return p
        except OSError:
            continue
    raise RuntimeError("không tìm được cổng trống")


def test_expand_targets():
    assert expand(["127.0.0.1:8101-8103"]) == [f"http://127.0.0.1:{p}" for p in (8101, 8102, 8103)]
    assert len(expand(["192.168.1.0/24"])) == 254
    assert expand(["10.0.0.5"]) == ["http://10.0.0.5"]
    with pytest.raises(ValueError):
        expand(["10.0.0.0/16"])


async def _farm(scale=4.0):
    port = _free_port_pair()
    farm = VRobotFarm(2, port, scale=scale)
    task = asyncio.create_task(farm.serve())
    await farm.wait_started()
    return farm, task, port


async def _connect(reg, port):
    found = await scan([f"127.0.0.1:{port}-{port + 1}"])
    assert len(found) == 2
    out = []
    for base, info in found:
        s = RobotSession(info["id"], info["name"], NetLink(base), "REAL", ip=base)
        reg.add(s)
        s.link.start()
        await reg.refresh(s)
        out.append(s)
    return out


def test_full_run_over_http():
    async def go():
        farm, task, port = await _farm()
        reg = Registry()
        coord = Coordinator(poll=0.02)
        reg.on_event = coord.on_event
        try:
            a, b = await _connect(reg, port)
            await reg.set_role(a.id, "leader")
            await reg.set_role(b.id, "follower")
            sh = P.shared_defaults()
            form, ferr, _ = P.formation(reg.cfg_params(a), reg.cfg_params(b), sh)
            plan = plan_path([[5, 3], [16, 3], [16, 10]], GRID, form, sh)
            assert not ferr and plan["ok"], plan["errors"]
            for s in (a, b):
                code, _ = await s.link.request("PUT", "/api/path", plan["path"])
                assert code == 200
            coord.start(plan["path"], a, b, sh)
            t0 = time.monotonic()
            while coord.state == "RUNNING" and time.monotonic() - t0 < 90:
                await asyncio.sleep(0.1)
            assert coord.state == "DONE", (coord.error, coord.log[-5:])
            assert farm.eng.collision_events == 0
            assert any("calib IMU" in e["msg"] for e in coord.log)
            assert a.link.errors == 0 and b.link.errors == 0
            assert b.last_tel["gap"] is not None
        finally:
            for s in list(reg.sessions.values()):
                s.link.close()
            farm.stop()
            await task
    asyncio.run(go())


def test_heartbeat_loss_stops_robot():
    """PC ngừng hỏi /api/state khi robot đang ARM → robot tự phanh, báo LINK_LOST."""
    async def go():
        farm, task, port = await _farm(scale=1.0)
        try:
            base = f"http://127.0.0.1:{port}"
            async with httpx.AsyncClient() as c:
                await c.put(f"{base}/api/role", json={"role": "leader"})
                sh = P.shared_defaults()
                form, _, _ = P.formation(P.defaults(), P.defaults(), sh)
                path = plan_path([[5, 3], [16, 3]], GRID, form, sh)["path"]
                await c.put(f"{base}/api/path", json=path)
                for _ in range(40):  # chờ auto-calib IMU xong
                    st = (await c.get(f"{base}/api/state")).json()
                    if st["tel"]["imu"]["ok"]:
                        break
                    await asyncio.sleep(0.1)
                r = (await c.post(f"{base}/api/cmd", json={"op": "arm"})).json()
                assert r["ok"], r
                await asyncio.sleep(1.0)  # không hỏi gì > link_timeout_ms (500 ms)
                st = (await c.get(f"{base}/api/state", params={"since": 0})).json()
                assert st["tel"]["state"] == "FAULT" and st["tel"]["fault"] == "LINK_LOST"
                assert any(e.get("e") == "fault" and e.get("code") == "LINK_LOST" for e in st["events"])
        finally:
            farm.stop()
            await task
    asyncio.run(go())

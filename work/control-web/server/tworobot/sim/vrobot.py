"""Robot ảo nói HTTP thật — đóng vai ESP32 trên LAN để kiểm chứng NetLink khi chưa có phần cứng.

Mỗi robot là một HTTP server riêng (cùng API mà firmware ESP32 phải cài, wiki/protocol.md §2),
chạy firmware ảo trong mô hình vật lý. Server Web Control nối tới bằng NetLink như robot thật.

    tworobot-vrobot --n 2 --port 8101          # robot ở http://127.0.0.1:8101 và :8102
    (trên Web Control: tab Thiết bị → Quét "127.0.0.1:8101-8102")

Khác robot thật: khi nhận PUT /api/path, robot ảo tự "được đặt" vào vị trí xuất phát
(thay cho người đặt xe), và /api/state có thêm trường truth (vị trí thật) để view so sánh.
"""
from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

import uvicorn
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import JSONResponse
from starlette.routing import Route

from .engine import SimEngine, SimRobot

NAMES = ("Robot HTTP A", "Robot HTTP B", "Robot HTTP C")


def make_app(eng: SimEngine, r: SimRobot) -> Starlette:
    async def handler(request: Request) -> JSONResponse:
        method, path = request.method, request.url.path
        if method == "GET":
            body = dict(request.query_params)
        else:
            raw = await request.body()
            try:
                body = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                return JSONResponse({"ok": False, "err": "JSON lỗi"}, status_code=400)
        if (method == "PUT" and path == "/api/path" and isinstance(body, dict)
                and r.fw.state not in ("RUNNING", "PAUSED", "JOG")):
            st = body.get("start") if r.fw.role == "leader" else (body.get("follower") or {}).get("start")
            if st:
                eng.place(r.id, st["x"], st["y"], st["yaw_deg"])
        code, res = r.fw.handle_request(method, path, body)
        if path == "/api/state" and code == 200 and res.get("tel") is not None:
            res["tel"]["truth"] = eng.truth(r.id)
            res["tel"]["sim_t"] = eng.t
        return JSONResponse(res, status_code=code)

    return Starlette(routes=[Route("/api/{p:path}", handler, methods=["GET", "PUT", "POST", "DELETE"])])


class VRobotFarm:
    def __init__(self, n: int = 2, port: int = 8101, host: str = "127.0.0.1", seed: int = 1,
                 scale: float = 1.0, state_dir: Path | None = None, size_mm=(3000, 1800)):
        self.eng = SimEngine(size_mm[0], size_mm[1], seed=seed, state_dir=state_dir)
        self.eng.scale = scale
        self.host = host
        self.ports = [port + k for k in range(n)]
        self.robots: list[SimRobot] = []
        for k in range(n):
            r = self.eng.add_robot(f"vr-{chr(97 + k)}", NAMES[k % len(NAMES)], 300, 300 + 400 * k, 0.0,
                                   {"motor_scale_r": 0.92} if k == 0 else {"motor_scale_l": 0.94})
            r.auto_hb = False          # heartbeat thật: từ các request HTTP của PC
            r.fw.link_watch = False
            self.robots.append(r)
        self.servers = [uvicorn.Server(uvicorn.Config(make_app(self.eng, r), host=host, port=p,
                                                      log_level="warning", lifespan="off"))
                        for r, p in zip(self.robots, self.ports)]

    async def serve(self) -> None:
        await asyncio.gather(self.eng.run_realtime(), *(s.serve() for s in self.servers))

    async def wait_started(self) -> None:
        while not all(s.started for s in self.servers):
            await asyncio.sleep(0.02)

    def stop(self) -> None:
        self.eng.stop()
        for s in self.servers:
            s.should_exit = True


def main() -> None:
    ap = argparse.ArgumentParser(description="Robot ảo nói HTTP (đóng vai ESP32 trên LAN)")
    ap.add_argument("--n", type=int, default=2)
    ap.add_argument("--port", type=int, default=8101)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--scale", type=float, default=1.0, help="hệ số thời gian mô phỏng")
    ap.add_argument("--state", default=None, help="thư mục 'flash' (mặc định: chỉ trong RAM)")
    a = ap.parse_args()
    farm = VRobotFarm(a.n, a.port, a.host, a.seed, a.scale, Path(a.state) if a.state else None)
    print("Robot HTTP ảo:", ", ".join(f"http://{a.host}:{p}" for p in farm.ports))
    try:
        asyncio.run(farm.serve())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

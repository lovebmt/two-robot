"""Corner coordinator (PC): chạy đường đi theo đoạn thẳng + quy trình 10 bước ở mỗi góc
(wiki/algorithms.md §5). Chỉ ra lệnh primitive và chờ sự kiện prim_done.
"""
from __future__ import annotations

import asyncio
import itertools
import time
from typing import Awaitable, Callable

from .registry import RobotSession

STEP_NAMES = [
    "Leader tới góc · follower bám rồi dừng",
    "Follower SNAPSHOT (đo mốc ρ, Δψ)",
    "Follower ALIGN (song song)",
    "Leader TURN 90°",
    "Leader EXIT (tiến ra khỏi góc)",
    "Follower tiến tới điểm cua (encoder)",
    "Follower TURN 90°",
    "Follower REACQUIRE (bắt lại leader)",
    "Follower ALIGN",
    "Đi tiếp đoạn sau",
]


class RunFault(Exception):
    pass


SNAP_HINT = {
    "NO_TARGET": "xe sau không thấy gì: xe trước quá xa, bị che, hoặc tấm đuôi không cùng độ cao mắt ToF",
    "ONE_EYE_ONLY": "chỉ 1 mắt thấy đuôi xe trước → xe sau lệch ngang: đặt lại thẳng hàng",
    "EYES_DISAGREE": "2 mắt đo chênh nhau quá nhiều → xe sau lệch ngang/lệch góc lớn so với xe trước, hoặc chưa Calib ToF",
}


async def preflight(L: RobotSession, F: RobotSession, form: dict, sh: dict,
                    run_prim: Callable[..., Awaitable[dict]]) -> list[dict]:
    """Kiểm tra trước khi chạy: 2 xe sẵn sàng, xe sau thấy xe trước đúng vị trí, phía trước trống.

    run_prim(sess, prim, **args) → bản tin prim_done (có ok/err/result). Trả danh sách mục kiểm tra.
    """
    checks: list[dict] = []

    def add(name: str, ok: bool, detail: str, warn: bool = False) -> None:
        checks.append({"name": name, "ok": ok, "detail": detail, "level": "ok" if ok and not warn else "warn" if ok else "err"})

    for s in (L, F):
        tel = s.last_tel or {}
        st = tel.get("state")
        add(f"{s.name}: kết nối, trạng thái", st not in (None, "FAULT", "ESTOP", "CALIB"),
            f"{st}" + (f" · lỗi {tel.get('fault')}" if tel.get("fault") else ""))
        add(f"{s.name}: pin", "BAT_LOW" not in tel.get("flags", []), f"{tel.get('vbat', 0):.1f} V")
        imu = tel.get("imu") or {}
        add(f"{s.name}: IMU đã calib", bool(imu.get("ok")),
            f"bias {imu.get('bias_dps', 0):+.2f}°/s" if imu.get("ok") else "chưa calib")

    msg = await run_prim(F, "SNAPSHOT")
    if not msg.get("ok"):
        add("Xe sau thấy đuôi xe trước bằng 2 mắt", False, SNAP_HINT.get(msg.get("err"), str(msg.get("err"))))
    else:
        r = msg["result"]
        add("Xe sau thấy đuôi xe trước bằng 2 mắt", True, f"trái {r['dl_mm']:.0f} mm · phải {r['dr_mm']:.0f} mm")
        g, gt, tol = r["rho"], form["g_target"], sh["start.gap_tol_mm"]
        move = "đẩy xe sau lên" if g > gt else "lùi xe sau ra"
        add("Khoảng cách 2 xe", abs(g - gt) <= tol,
            f"{g:.0f} mm (cần {gt:.0f} ± {tol:.0f})" + ("" if abs(g - gt) <= tol else f" → {move} {abs(g - gt):.0f} mm"))
        d = r["dpsi_deg"]
        ok = abs(d) <= sh["start.max_dpsi_deg"]
        add("Xe sau thẳng hàng với xe trước", ok,
            f"lệch {d:+.1f}°" + ("" if ok else f" (> {sh['start.max_dpsi_deg']:.0f}°): xoay xe sau lại")
            + (" → sẽ tự căn (ALIGN)" if ok and abs(d) > sh["start.align_dpsi_deg"] else ""),
            warn=ok and abs(d) > sh["start.align_dpsi_deg"])
    tof = (L.last_tel or {}).get("tof") or {}
    near = [v["mm"] for v in tof.values() if v.get("st") == 0]
    dmin = min(near) if near else None
    add("Phía trước xe đầu trống", dmin is None or dmin >= sh["start.front_clear_mm"],
        "không thấy vật cản" if dmin is None else f"vật gần nhất {dmin:.0f} mm (cần ≥ {sh['start.front_clear_mm']:.0f})")
    return checks


class Coordinator:
    def __init__(self, clock: Callable[[], float] = time.monotonic, poll: float = 0.02,
                 on_change: Callable[[], None] = lambda: None,
                 on_mark: Callable[[str, int], None] = lambda tag, k: None):
        self.clock = clock
        self.poll = poll
        self.on_change = on_change
        self.on_mark = on_mark
        self.state = "IDLE"  # IDLE | RUNNING | PAUSED | FAULT | DONE | ABORTED
        self.error: str | None = None
        self.seg = 0
        self.step = 0
        self.path: dict | None = None
        self.L: RobotSession | None = None
        self.F: RobotSession | None = None
        self.log: list[dict] = []
        self._task: asyncio.Task | None = None
        self._ids = itertools.count(1)
        self._waiters: dict[int, asyncio.Future] = {}
        self._acks: dict[int, asyncio.Future] = {}
        self._calib: dict[str, asyncio.Future] = {}
        self.phase = ""
        self.checks: list[dict] = []
        self._fault: str | None = None
        self.paused = False

    # ------------------------------------------------------------ status
    def status(self) -> dict:
        n_corners = len(self.path["corners_s"]) if self.path else 0
        return {"state": "PAUSED" if self.paused and self.state == "RUNNING" else self.state,
                "error": self.error, "seg": self.seg, "step": self.step, "phase": self.phase, "checks": self.checks,
                "n_segments": len(self.path["segments"]) if self.path else 0,
                "n_corners": n_corners, "steps": STEP_NAMES,
                "path_id": self.path["path_id"] if self.path else None,
                "log": self.log[-12:]}

    def _set(self, seg: int, step: int, msg: str | None = None) -> None:
        self.seg, self.step = seg, step
        if msg:
            self.log.append({"t": round(self.clock(), 2), "msg": msg})
        self.on_change()

    # ------------------------------------------------------------ events
    def on_event(self, sess: RobotSession, msg: dict) -> None:
        if msg.get("t") in ("ack", "nack"):
            fut = self._acks.pop(msg.get("seq"), None)
            if fut is not None and not fut.done():
                fut.set_result(msg)
            return
        if msg.get("e") == "calib_done":
            fut = self._calib.pop(sess.id, None)
            if fut is not None and not fut.done():
                fut.set_result(msg)
            return
        if msg.get("e") == "prim_done":
            fut = self._waiters.pop(int(msg.get("id", -1)), None)
            if fut is not None and not fut.done():
                fut.set_result(msg)
        elif msg.get("e") == "fault" and self.state == "RUNNING":
            self._fault = f"{sess.name}: {msg.get('code')}"

    # ------------------------------------------------------------ primitives
    async def _cmd(self, sess: RobotSession, op: str, **kw) -> None:
        await sess.link.send({"t": "cmd", "op": op, **kw})

    async def _cmd_ack(self, sess: RobotSession, op: str, timeout: float = 3.0, **kw) -> None:
        """Gửi lệnh và chờ ack; nack → RunFault."""
        seq = 100000 + next(self._ids)
        fut = asyncio.get_running_loop().create_future()
        self._acks[seq] = fut
        await sess.link.send({"t": "cmd", "op": op, "seq": seq, **kw})
        elapsed, last = 0.0, self.clock()
        while not fut.done():
            await asyncio.sleep(self.poll)
            now = self.clock()
            elapsed += now - last
            last = now
            if elapsed > timeout:
                self._acks.pop(seq, None)
                raise RunFault(f"{sess.name}: không phản hồi lệnh {op}")
        res = fut.result()
        if res.get("t") == "nack":
            raise RunFault(f"{sess.name} từ chối {op}: {res.get('err')}")

    @staticmethod
    def _cfg(sess: RobotSession, key: str, default):
        return ((sess.config or {}).get("params") or {}).get(key, default)

    async def _calib_imu(self, sessions: list[RobotSession]) -> None:
        """Calib gyro đồng thời các xe (đứng yên), chờ kết quả, ghi log bias."""
        loop = asyncio.get_running_loop()
        pending = []
        for s in sessions:
            fut = loop.create_future()
            self._calib[s.id] = fut
            await self._cmd_ack(s, "calib_imu")
            pending.append((s, fut))
        for s, fut in pending:
            t = float(self._cfg(s, "imu.calib_time_s", 2.0))
            res = await self._wait(fut, 8 * t + 5, f"{s.name} calib IMU")
            self.log.append({"t": round(self.clock(), 2),
                             "msg": f"{s.name}: calib IMU bias {res['bias_dps']:+.2f}°/s, nhiễu {res['std_dps']:.3f}°/s"
                                    + (f" (làm lại {res['restarts']} lần do bị chạm)" if res.get("restarts") else "")})
        self.on_change()

    async def _start_prim(self, sess: RobotSession, prim: str, **args) -> asyncio.Future:
        pid = next(self._ids)
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self._waiters[pid] = fut
        seq = 200000 + pid
        ack = loop.create_future()
        self._acks[seq] = ack
        await sess.link.send({"t": "cmd", "op": "prim", "seq": seq, "id": pid, "prim": prim, **args})
        # SimLink/NetLink trả ack ngay trong lúc gửi; robot từ chối → báo lỗi ngay, không chờ timeout.
        if ack.done() and ack.result().get("t") == "nack":
            self._waiters.pop(pid, None)
            raise RunFault(f"{sess.name} từ chối {prim}: {ack.result().get('err')}")
        self._acks.pop(seq, None)
        return fut

    async def _wait(self, fut: asyncio.Future, timeout: float, what: str) -> dict:
        elapsed, last = 0.0, self.clock()
        while not fut.done():
            await asyncio.sleep(self.poll)
            now = self.clock()
            if not self.paused:
                elapsed += now - last
            last = now
            if self._fault:
                raise RunFault(self._fault)
            if elapsed > timeout:
                raise RunFault(f"timeout {timeout:.0f}s: {what}")
        res = fut.result()
        if not res.get("ok"):
            raise RunFault(f"{what}: {res.get('err')}")
        return res.get("result", {})

    async def _wait_until(self, pred: Callable[[], bool], timeout: float) -> bool:
        elapsed, last = 0.0, self.clock()
        while not pred():
            await asyncio.sleep(self.poll)
            now = self.clock()
            elapsed += now - last
            last = now
            if elapsed > timeout:
                return False
        return True

    async def _prim_raw(self, sess: RobotSession, prim: str, timeout: float = 20.0, **args) -> dict:
        """Chạy primitive, trả nguyên bản tin prim_done (không ném lỗi khi ok=False)."""
        fut = await self._start_prim(sess, prim, **args)
        elapsed, last = 0.0, self.clock()
        while not fut.done():
            await asyncio.sleep(self.poll)
            now = self.clock()
            elapsed += 0.0 if self.paused else now - last
            last = now
            if elapsed > timeout:
                raise RunFault(f"timeout: {sess.name} {prim}")
        return fut.result()

    async def _prim(self, sess: RobotSession, prim: str, timeout: float | None = None, **args) -> dict:
        fut = await self._start_prim(sess, prim, **args)
        return await self._wait(fut, timeout or self._timeout, f"{sess.name} {prim}")

    async def _wait_still(self, sess: RobotSession, hold_s: float, timeout: float) -> None:
        elapsed, still, last = 0.0, 0.0, self.clock()
        while True:
            await asyncio.sleep(self.poll)
            now = self.clock()
            dt = now - last
            last = now
            if self.paused:
                continue
            elapsed += dt
            tel = sess.last_tel or {}
            v = abs(tel.get("est", {}).get("v", 99.0))
            still = still + dt if v < 3.0 else 0.0
            if still >= hold_s:
                return
            if self._fault:
                raise RunFault(self._fault)
            if elapsed > timeout:
                raise RunFault(f"timeout: {sess.name} không dừng")

    # ------------------------------------------------------------ control
    def start(self, path: dict, leader: RobotSession, follower: RobotSession, shared: dict) -> None:
        if self._task and not self._task.done():
            raise RuntimeError("đang chạy")
        self.path, self.L, self.F, self.shared = path, leader, follower, shared
        self.state, self.error, self._fault, self.paused = "RUNNING", None, None, False
        self.log = []
        self._timeout = float(shared["corner.step_timeout_s"])
        self._task = asyncio.get_running_loop().create_task(self._run())

    async def pause(self) -> None:
        if self.state == "RUNNING":
            self.paused = True
            for s in (self.L, self.F):
                await self._cmd(s, "pause")
            self.on_change()

    async def resume(self) -> None:
        if self.state == "RUNNING" and self.paused:
            self.paused = False
            for s in (self.L, self.F):
                await self._cmd(s, "resume")
            self.on_change()

    async def stop(self, state: str = "ABORTED") -> None:
        if self._task and not self._task.done():
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
        for s in (self.L, self.F):
            if s is not None:
                try:
                    await self._cmd(s, "stop")
                except Exception:  # noqa: BLE001
                    pass
        if self.state in ("RUNNING", "FAULT"):
            self.state = state
        self.paused = False
        self.on_change()

    async def wait_done(self) -> None:
        if self._task:
            await asyncio.shield(self._task)

    # ------------------------------------------------------------ main
    async def _run(self) -> None:
        p, L, F = self.path, self.L, self.F
        form, sh = p["formation"], self.shared
        segs = p["segments"]
        v = max(sh["motion.v_cruise"], 20.0)
        try:
            need = [s for s in (L, F) if self._cfg(s, "imu.recal_before_run", True)
                    or not ((s.last_tel or {}).get("imu") or {}).get("ok")]
            if need:
                self.phase = "CALIB_IMU"
                self._set(0, 0, "Calib IMU: giữ 2 xe đứng yên…")
                await self._calib_imu(need)
            self.checks = []
            if sh.get("start.check_enable", True):
                self.phase = "CHECK"
                self._set(0, 0, "Kiểm tra trước khi chạy…")
                # Telemetry 25 Hz có thể còn là bản trước khi calib xong → chờ bản mới.
                await self._wait_until(lambda: all((s.last_tel or {}).get("state") not in (None, "CALIB")
                                                   for s in (L, F)), 2.0)
                self.checks = await preflight(L, F, form, sh, self._prim_raw)
                bad = [c for c in self.checks if not c["ok"]]
                if bad:
                    raise RunFault("Kiểm tra trước khi chạy KHÔNG đạt: "
                                   + "; ".join(f"{c['name']}: {c['detail']}" for c in bad))
                self.log.append({"t": round(self.clock(), 2), "msg": "Kiểm tra trước khi chạy: đạt"})
            self.phase = "RUN"
            for s in (L, F):
                await self._cmd_ack(s, "arm", path_id=p["path_id"])
            if any(c["name"].startswith("Xe sau thẳng hàng") and c["level"] == "warn" for c in self.checks):
                await self._prim(F, "ALIGN")
                self.log.append({"t": round(self.clock(), 2), "msg": "Xe sau đã tự căn thẳng với xe trước"})
            for i, seg in enumerate(segs):
                last = i == len(segs) - 1
                s_end = p["total_len"] if last else p["corners_s"][i]
                self._set(i, 1, f"Đoạn {i + 1}/{len(segs)}: leader chạy tới s={s_end:.0f}")
                fut_l = await self._start_prim(L, "DRIVE_TO", s_end=s_end, heading_deg=seg["heading_deg"])
                await self._start_prim(F, "FOLLOW", g_target=form["g_target"], gap_min_safe=form["gap_min_safe"])
                t_seg = seg["len"] / (0.5 * v) + 30
                res_l = await self._wait(fut_l, t_seg, f"{L.name} DRIVE_TO")
                await self._wait_still(F, 0.6, t_seg)
                await self._prim(F, "HOLD")
                if last:
                    break
                nxt = segs[i + 1]

                self._set(i, 2)
                snap = await self._prim(F, "SNAPSHOT")
                self._set(i, 3, f"Góc {i + 1}: ρ={snap['rho']:.0f} mm, Δψ={snap['dpsi_deg']:.1f}°")
                if abs(snap["dpsi_deg"]) > sh["corner.align_tol_deg"]:
                    await self._prim(F, "ALIGN")
                    snap = await self._prim(F, "SNAPSHOT")
                if snap["rho"] < form["g_hold_min"] - 5:
                    raise RunFault(f"follower quá gần ({snap['rho']:.0f} < {form['g_hold_min']:.0f} mm) để leader quay")

                self._set(i, 4)
                self.on_mark("L_turn", i)
                await self._prim(L, "TURN", heading_deg=nxt["heading_deg"])
                self._set(i, 5)
                await self._prim(L, "DRIVE_TO", s_end=res_l["s"] + form["exit_dist"], heading_deg=nxt["heading_deg"])

                self._set(i, 6)
                await self._prim(F, "DRIVE_TO", s_end=snap["s"] + snap["rho"])
                await self._cmd(F, "set_s", s=res_l["s"])
                self.on_mark("F_turn", i)
                self._set(i, 7)
                await self._prim(F, "TURN", deg=seg["turn_after"])
                self._set(i, 8)
                acq = await self._prim(F, "REACQUIRE", rho_exp=form["exit_dist"])
                self._set(i, 9, f"Góc {i + 1}: bắt lại leader ρ={acq['rho']:.0f} mm (lệch quét {acq['off_deg']:+.0f}°)")
                al = await self._prim(F, "ALIGN")
                self._set(i, 10, f"Góc {i + 1}: xong, Δψ còn {al['dpsi_deg']:.1f}°")
            for s in (L, F):
                await self._cmd(s, "stop")
            self.state = "DONE"
            self._set(self.seg, self.step, "Hoàn thành đường đi")
        except RunFault as e:
            self.state, self.error = "FAULT", str(e)
            self.log.append({"t": round(self.clock(), 2), "msg": f"LỖI: {e}"})
            for s in (L, F):
                try:
                    await self._cmd(s, "pause")
                except Exception:  # noqa: BLE001
                    pass
            self.on_change()
        finally:
            for fut in [*self._waiters.values(), *self._calib.values()]:
                if not fut.done():
                    fut.cancel()
            self._waiters.clear()
            self._calib.clear()
            self.phase = ""

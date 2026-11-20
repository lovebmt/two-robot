"""VirtualFirmware: ứng dụng firmware (lib/app) chạy trên HAL bất kỳ.

- tick(dt) gọi 100 Hz (task `control`): swap tham số, đo, ước lượng, primitive, PWM, an toàn, telemetry 25 Hz.
- on_range(...) gọi khi có mẫu ToF (task `range`, ~30 Hz).
- handle_request(method, path, body): HTTP API (wiki/protocol.md §2).
- handle_ws(msg): lệnh WebSocket (wiki/protocol.md §3). Đầu ra qua self.emit(msg).
"""
from __future__ import annotations

import math
from typing import Callable

from .. import params as P
from .control import DriveBase, HeadingEstimator, ImuCalibrator, Odometry
from .gapkf import GapKF
from .hal import Hal
from .paramsvc import ParamService
from .primitives import PRIMS, Prim
from .rangeproc import RangeMeas, RangeProcessor

FW_VERSION = "0.1.0-py"

# Trạng thái: IDLE → (PUT path, arm) → ARMED ⇄ RUNNING (có primitive) ; PAUSED ; JOG ; FAULT ; ESTOP
RUN_STATES = ("ARMED", "RUNNING", "PAUSED")


class VirtualFirmware:
    TEL_PERIOD = 0.04

    def __init__(self, robot_id: str, name: str, hal: Hal):
        self.id = robot_id
        self.name = name
        self.hal = hal
        self.emit: Callable[[dict], None] = lambda msg: None
        self.params = ParamService(hal)
        nvs = hal.nvs_load() or {}
        self.role: str | None = nvs.get("role")
        self.state = "IDLE"
        self.paused = False
        self.fault: str | None = None
        self.drive = DriveBase()
        self.heading = HeadingEstimator()
        self.odo = Odometry()
        self.rangep = RangeProcessor()
        self.kf = GapKF()
        self.meas: RangeMeas | None = None
        self.raw_tof = {"l": (0.0, 4), "r": (0.0, 4)}
        self.path: dict | None = None
        self.formation: dict = {}
        self.prim: Prim | None = None
        self.relay: tuple[int, float, float] | None = None  # (t_ms, v_L, s_L)
        self.last_hb_ms = 0
        self.link_watch = False
        self.jog = (0.0, 0.0, 0)
        self.ident_until = 0
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.vbat = 12.0
        self._last_ticks: tuple[int, int] | None = None
        self._tel_acc = 0.0
        self.flags: set[str] = set()
        # IMU: calib bias gyro khi đứng yên (bắt buộc trước ARM nếu imu.require_calib)
        self.imu_cal: ImuCalibrator | None = None
        self.imu_ok = False
        self.imu_info: dict = {}
        self._calib_prev_state = "IDLE"
        self._enc_still_t = 0.0

    # ------------------------------------------------------------ control
    def tick(self, dt: float) -> None:
        groups = self.params.swap_if_pending()
        if groups:
            self._on_params(groups)
        cfg, der = self.params.active, self.params.derived
        now = self.hal.now_ms()

        tl, tr = self.hal.read_encoder("l"), self.hal.read_encoder("r")
        if self._last_ticks is None:
            self._last_ticks = (tl, tr)
        moved = (tl, tr) != self._last_ticks
        dl = (tl - self._last_ticks[0]) * der["mm_per_tick_l"]
        dr = (tr - self._last_ticks[1]) * der["mm_per_tick_r"]
        self._last_ticks = (tl, tr)
        self.drive.measure(dl, dr, dt)

        gz = self.hal.read_gyro_z()
        enc_still = self.drive.braking and not moved
        self._enc_still_t = self._enc_still_t + dt if enc_still else 0.0
        calibrating = self.imu_cal is not None
        if calibrating:
            res = self.imu_cal.feed(gz, moved, dt)
            if res is not None:
                self._finish_calib(res)
        elif (not self.imu_ok and cfg["imu.auto_calib"] and self.state == "IDLE"
              and self._enc_still_t > 0.5):
            self.start_calib()
        still = enc_still and abs(self.drive.v_meas_l) < 1.0 and abs(self.drive.v_meas_r) < 1.0
        straight = abs(self.w_cmd) < 0.05 and abs(self.v_cmd) > 5.0
        # Chưa calib mà xe đứng yên: bias chưa biết → không tích phân hướng (tránh trôi lúc khởi động).
        hold = calibrating or (not self.imu_ok and enc_still)
        self.heading.update(gz, dl, dr, straight, still, cfg, dt, calibrating=hold,
                            temp=self.hal.read_imu_temp())
        self.odo.update(dl, dr, self.heading.psi)

        self.vbat = self.hal.read_vbat()
        self.flags.discard("BAT_LOW")
        if self.vbat < cfg["power.bat_min_v"]:
            self.flags.add("BAT_LOW")
        if (self.state in RUN_STATES and self.link_watch
                and now - self.last_hb_ms > cfg["safety.link_timeout_ms"]):
            self.raise_fault("LINK_LOST")

        v, w, brake = 0.0, 0.0, True
        if self.state == "RUNNING" and self.prim is not None and not self.paused:
            v, w, brake = self.prim.update(self, dt)
            if self.prim is not None and self.prim.done:
                self._finish_prim()
        elif self.state == "JOG":
            if now < self.jog[2]:
                v, w, brake = self.jog[0], self.jog[1], False
            else:
                self.state = "IDLE"
        self.v_cmd, self.w_cmd = (0.0, 0.0) if brake else (v, w)
        duty_l, duty_r, b = self.drive.command(v, w, brake, cfg, self.vbat, dt)
        self.hal.set_motor("l", duty_l, b)
        self.hal.set_motor("r", duty_r, b)

        self._tel_acc += dt
        if self._tel_acc >= self.TEL_PERIOD - 1e-9:
            self._tel_acc -= self.TEL_PERIOD
            self.emit(self.telemetry())

    def on_range(self, l_mm: float, l_st: int, r_mm: float, r_st: int) -> None:
        self.raw_tof = {"l": (l_mm, l_st), "r": (r_mm, r_st)}
        m = self.rangep.process(l_mm, l_st, r_mm, r_st, self.params.active,
                                self.formation.get("leader_rear_mm", self.params.active["body.axle_to_rear_mm"]))
        m.t_ms = self.hal.now_ms()
        self.meas = m
        if self.prim is not None and self.state == "RUNNING" and not self.paused:
            self.prim.on_range(self, m)

    def _on_params(self, groups: set[str]) -> None:
        """Module phản ứng khi tham số đổi (config-management §3.3)."""
        if "range" in groups or "body" in groups:
            self.rangep.reset()
            if self.kf.active and self.meas is not None and self.meas.valid:
                self.kf.reset(self.meas.rho, self.kf.vL, self.params.active["range.sigma_mm"])
        # PI bánh/hướng/follow: tích phân lưu theo đơn vị đầu ra → tự bumpless.
        # Odometry: giữ pose, chu kỳ sau dùng mm_per_tick mới.

    # ------------------------------------------------------------ IMU calib
    def start_calib(self) -> bool:
        if self.state not in ("IDLE", "ARMED", "FAULT", "CALIB"):
            return False
        if self.imu_cal is None:
            self._calib_prev_state = self.state
        self.imu_cal = ImuCalibrator(self.params.active)
        self.state = "CALIB"
        return True

    def _finish_calib(self, res: dict) -> None:
        self.imu_cal = None
        if self.state == "CALIB":
            self.state = self._calib_prev_state
        cfg = self.params.active
        out = {"restarts": res.get("restarts", 0)}
        if res["ok"]:
            self.heading.bias = res["bias"]
            self.heading.anchor_bias()
            self.imu_ok = True
            out.update(bias_dps=math.degrees(res["bias"]), std_dps=math.degrees(res["std"]),
                       spread_dps=math.degrees(res["spread"]), n=res["n"])
            self.imu_info = {**out, "t_ms": self.hal.now_ms()}
            self.flags.discard("IMU_BIAS_HIGH")
            if abs(out["bias_dps"]) > cfg["imu.bias_max_dps"]:
                self.flags.add("IMU_BIAS_HIGH")
        else:
            self.imu_ok = False
        self.emit({"t": "evt", "e": "calib_done", "ok": res["ok"], "err": res.get("err"),
                   "result": out, "ts": self.hal.now_ms()})

    # ------------------------------------------------------------ helpers
    def raise_fault(self, code: str) -> None:
        if self.state in ("FAULT", "ESTOP"):
            return
        if self.prim is not None:
            self.emit({"t": "evt", "e": "prim_done", "id": self.prim.id, "prim": self.prim.name,
                       "ok": False, "err": code, "result": {}})
            self.prim = None
        self.fault = code
        self.state = "FAULT"
        self.emit({"t": "evt", "e": "fault", "code": code, "ts": self.hal.now_ms()})

    def _finish_prim(self) -> None:
        p = self.prim
        self.prim = None
        self.kf.active = False if p.name == "FOLLOW" else self.kf.active
        if self.state == "RUNNING":
            self.state = "ARMED"
        self.emit({"t": "evt", "e": "prim_done", "id": p.id, "prim": p.name, "ok": p.ok,
                   "err": p.err, "result": p.result, "ts": self.hal.now_ms()})

    def _set_pose(self, x: float, y: float, yaw_deg: float, s: float) -> None:
        self.odo.reset(x, y, s)
        self.heading.psi = math.radians(yaw_deg)
        self.kf.active = False
        self.rangep.reset()
        self._last_ticks = None

    def _ack(self, msg: dict, ok: bool = True, err: str | None = None, **extra) -> None:
        out = {"t": "ack" if ok else "nack", "seq": msg.get("seq")}
        if err:
            out["err"] = err
        out.update(extra)
        self.emit(out)

    # ------------------------------------------------------------ telemetry
    def telemetry(self) -> dict:
        cfg = self.params.active
        m = self.meas
        now = self.hal.now_ms()
        flags = set(self.flags) | self.params.flags
        if self.drive.sat:
            flags.add("SAT")
        if now < self.ident_until:
            flags.add("IDENT")
        if abs(self.heading.w_enc - self.heading.w_gyro) > 0.35:
            flags.add("SLIP")
        if not self.imu_ok:
            flags.add("IMU_UNCAL")
        if self.imu_cal is not None:
            flags.add("IMU_CALIB")
        if self.params.dirty():
            flags.add("CFG_DIRTY")
        return {
            "t": "tel", "id": self.id, "t_ms": now, "state": "PAUSED" if self.paused and self.state in RUN_STATES else self.state,
            "role": self.role, "cfg_rev": self.params.rev, "fault": self.fault,
            "est": {"x": self.odo.x, "y": self.odo.y, "yaw": self.heading.psi, "s": self.odo.s,
                    "v": self.drive.v_meas, "w": self.heading.w},
            "wheels": {
                "l": {"v_cmd": self.drive.wl.v_ref, "v": self.drive.v_meas_l, "duty": self.drive.wl.duty,
                      "adapt": self.drive.wl.adapt},
                "r": {"v_cmd": self.drive.wr.v_ref, "v": self.drive.v_meas_r, "duty": self.drive.wr.duty,
                      "adapt": self.drive.wr.adapt},
            },
            "tof": {"l": {"mm": self.raw_tof["l"][0], "st": self.raw_tof["l"][1]},
                    "r": {"mm": self.raw_tof["r"][0], "st": self.raw_tof["r"][1]}},
            "gap": {"est": self.kf.g if self.kf.active else None,
                    "meas": m.rho if (m and m.valid) else None,
                    "target": self.prim.g_t if (self.prim is not None and self.prim.name == "FOLLOW") else None,
                    "vL": self.kf.vL if self.kf.active else None},
            "dpsi": m.dpsi if (m and m.dpsi is not None) else None,
            "prim": {"id": self.prim.id, "name": self.prim.name} if self.prim else None,
            "flags": sorted(flags), "vbat": self.vbat, "gyro_bias": self.heading.bias,
            "imu": {"ok": self.imu_ok, "bias_dps": math.degrees(self.heading.bias),
                    "temp_c": self.heading.temp,
                    "k_dps_c": math.degrees(self.heading.k(cfg)),
                    "k_learned": self.heading.k_temp is not None,
                    "calib": self.imu_cal.progress if self.imu_cal else None,
                    "std_dps": self.imu_info.get("std_dps"), "zupt": self.heading.zupt_active,
                    "age_s": (now - self.imu_info["t_ms"]) / 1000 if "t_ms" in self.imu_info else None},
            "geom": {"length": cfg["body.length_mm"], "width": cfg["body.width_mm"],
                     "front": cfg["body.axle_to_front_mm"], "rear": cfg["body.axle_to_rear_mm"],
                     "tof_x": cfg["range.x_mm"], "tof_b": cfg["range.baseline_mm"],
                     "track": cfg["wheel.track_mm"], "wheel_d": cfg["wheel.diameter_mm"]},
        }

    # ------------------------------------------------------------ HTTP
    def info(self) -> dict:
        return {"id": self.id, "name": self.name, "fw": FW_VERSION, "role": self.role,
                "state": self.state, "cfg_rev": self.params.rev, "schema_ver": P.SCHEMA["schema_ver"]}

    def handle_request(self, method: str, path: str, body: dict | None = None) -> tuple[int, dict]:
        body = body or {}
        running = self.state in ("RUNNING", "PAUSED", "JOG")
        if method == "GET" and path == "/api/info":
            return 200, self.info()
        if path == "/api/config":
            if method == "GET":
                ps = self.params
                return 200, {"params": ps.view(), "derived": ps.view_derived(), "cfg_rev": ps.rev,
                             "crc": P.crc(ps.view()), "saved_crc": ps.saved_crc, "dirty": ps.dirty(),
                             "flags": sorted(ps.flags)}
            if method == "PUT":
                res = self.params.submit(body.get("patch", {}), running)
                if res.get("ok") and body.get("persist"):
                    res["saved"] = self.params.save()
                return (200 if res.get("ok") else 409 if res.get("busy") else 400), res
        if method == "POST" and path == "/api/config/save":
            return 200, self.params.save()
        if method == "POST" and path == "/api/config/reload":
            res = self.params.reload(running)
            return (200 if res.get("ok") else 400), res
        if method == "POST" and path == "/api/config/factory_reset":
            res = self.params.factory_reset(running)
            return (200 if res.get("ok") else 400), res
        if method == "PUT" and path == "/api/role":
            role = body.get("role")
            if role not in ("leader", "follower", None):
                return 400, {"ok": False, "err": "role không hợp lệ"}
            if running:
                return 409, {"ok": False, "err": "đang chạy"}
            self.role = role
            self.hal.nvs_save({"role": role})
            return 200, {"ok": True, "role": role}
        if method == "PUT" and path == "/api/path":
            return self._load_path(body, running)
        if method == "POST" and path == "/api/identify":
            self.ident_until = self.hal.now_ms() + 2500
            return 200, {"ok": True}
        return 404, {"ok": False, "err": f"không có route {method} {path}"}

    def _load_path(self, path: dict, running: bool) -> tuple[int, dict]:
        if running:
            return 409, {"ok": False, "err": "đang chạy"}
        if self.role not in ("leader", "follower"):
            return 400, {"ok": False, "err": "chưa có vai trò"}
        self.path = path
        self.formation = dict(path.get("formation", {}))
        if self.role == "leader":
            st, s0 = path["start"], 0.0
        else:
            st, s0 = path["follower"]["start"], float(path["follower"]["start_s"])
        self._set_pose(st["x"], st["y"], st["yaw_deg"], s0)
        if self.state in ("FAULT",):
            self.state, self.fault = "IDLE", None
        return 200, {"ok": True, "path_id": path.get("path_id"), "crc": path.get("crc32")}

    # ------------------------------------------------------------ WS
    def handle_ws(self, msg: dict) -> None:
        t = msg.get("t")
        now = self.hal.now_ms()
        if t == "hb":
            self.last_hb_ms = now
            self.link_watch = True
            return
        if t == "relay":
            self.relay = (now, float(msg.get("v", 0.0)), float(msg.get("s", 0.0)))
            return
        if t == "cfg_set":
            res = self.params.submit({msg["key"]: msg["value"]}, self.state in ("RUNNING", "PAUSED", "JOG"))
            self._ack(msg, res.get("ok", False), None if res.get("ok") else str(res.get("errors")), result=res)
            return
        if t != "cmd":
            return
        op = msg.get("op")
        if op == "estop":
            self.prim, self.paused, self.imu_cal = None, False, None
            self.state = "ESTOP"
            self._ack(msg)
            return
        if op == "reset":
            if self.state in ("ESTOP", "FAULT"):
                self.state, self.fault, self.paused, self.prim = "IDLE", None, False, None
            self._ack(msg)
            return
        if self.state == "ESTOP":
            self._ack(msg, False, "estop")
            return
        if op == "arm":
            if self.path is None:
                return self._ack(msg, False, "no_path")
            if self.state == "FAULT":
                return self._ack(msg, False, f"fault:{self.fault}")
            if "CFG_DEFAULT" in self.params.flags:
                return self._ack(msg, False, "cfg_default")
            if self.imu_cal is not None:
                return self._ack(msg, False, "imu_calibrating")
            if self.params.active["imu.require_calib"] and not self.imu_ok:
                return self._ack(msg, False, "imu_uncal")
            self.state, self.paused = "ARMED", False
            self.last_hb_ms = now
            return self._ack(msg)
        if op == "prim":
            if self.state not in RUN_STATES:
                return self._ack(msg, False, f"state:{self.state}")
            cls = PRIMS.get(msg.get("prim"))
            if cls is None:
                return self._ack(msg, False, "unknown_prim")
            if self.prim is not None:
                old = self.prim
                self.emit({"t": "evt", "e": "prim_done", "id": old.id, "prim": old.name, "ok": False,
                           "err": "preempted", "result": {}, "ts": now})
            args = {k: v for k, v in msg.items() if k not in ("t", "op", "id", "prim", "seq")}
            p = cls(int(msg.get("id", 0)), args)
            self.prim = p
            self.state = "RUNNING"
            p.start(self)
            self._ack(msg)
            if p.done:
                self._finish_prim()
            return
        if op == "pause":
            self.paused = True
            return self._ack(msg)
        if op == "resume":
            self.paused = False
            self.last_hb_ms = now
            return self._ack(msg)
        if op == "stop":
            self.prim, self.paused, self.imu_cal = None, False, None
            if self.state != "FAULT":
                self.state = "IDLE"
            return self._ack(msg)
        if op == "calib_imu":
            ok = self.start_calib()
            return self._ack(msg, ok, None if ok else f"state:{self.state}")
        if op == "set_s":
            self.odo.s = float(msg["s"])
            return self._ack(msg)
        if op == "jog":
            if self.state not in ("IDLE", "JOG"):
                return self._ack(msg, False, f"state:{self.state}")
            self.state = "JOG"
            self.jog = (float(msg.get("v", 0.0)), float(msg.get("w", 0.0)), now + 300)
            return self._ack(msg)
        self._ack(msg, False, "unknown_op")

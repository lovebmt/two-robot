#!/usr/bin/env python3
"""Sinh dữ liệu đối chiếu Python ↔ C++ (test/vectors/*.json) từ firmware ảo + mô phỏng.

    work/control-web/.venv/bin/python work/firmware/tools/gen_vectors.py
Test native `test_tof_fit_matches_python` so kết quả C++ với giá trị Python ở đây.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "control-web" / "server"))

from tworobot.fw.primitives import TofCal, tof_fit  # noqa: E402
from tworobot.sim.engine import SimEngine  # noqa: E402


def tof_cases() -> list[dict]:
    cases = []
    for k, (mm, yaw0, dist) in enumerate([
        ({"tof_dx_l": 2.0, "tof_off_r": -6.0}, 4.0, 200),
        ({"tof_dx_r": -3.0, "tof_off_l": 8.0, "tof_db": 4.0}, -7.0, 160),
        ({"tof_dx_l": 1.5, "tof_off_l": -5, "tof_off_r": 7, "tof_yaw_l": 3.0}, 0.0, 260),
    ]):
        eng = SimEngine(3000, 1800, seed=11 + k)
        r = eng.add_robot("a", "a", 3000 + eng.world.wall_margin - 120 - dist, 900, math.radians(yaw0), mm)
        eng.advance(3.0)
        r.fw.handle_ws({"t": "cmd", "op": "prim", "id": 1, "prim": "TOF_CAL"})
        prim = r.fw.prim
        assert isinstance(prim, TofCal)
        while r.fw.prim is prim:
            eng.advance(0.1)
        samples = [[round(th, 6), round(dl, 3), round(dr, 3)] for th, dl, dr in prim.samples]
        expect = tof_fit([tuple(s) for s in samples])
        assert expect["ok"], expect
        cases.append({"samples": samples,
                      "expect": {k2: expect[k2] for k2 in ("c_mm", "theta0_deg", "baseline_mm", "d_perp_mm")}})
    return cases


def api_contract() -> dict:
    """Tập khoá JSON mà server (NetLink/UI) đọc từ robot — lấy từ firmware ảo Python."""
    eng = SimEngine(3000, 1800, seed=1)
    r = eng.add_robot("a", "a", 500, 500, 0.0)
    eng.advance(3.0)

    def keys(d, prefix=""):
        out = []
        for k, v in d.items():
            out.append(prefix + k)
            if isinstance(v, dict) and k not in ("result",):
                out += keys(v, prefix + k + ".")
        return out
    _, st = r.fw.handle_request("GET", "/api/state", {"since": 0})
    _, info = r.fw.handle_request("GET", "/api/info")
    _, cfg = r.fw.handle_request("GET", "/api/config")
    tel = st["tel"]
    for k in ("truth", "sim_t"):
        tel.pop(k, None)
    return {"state": sorted(set(keys({k: v for k, v in st.items() if k != "tel"}))),
            "tel": sorted(set(keys(tel))), "info": sorted(info), "config": sorted(k for k in cfg if k != "params"),
            "calib_event": sorted(keys(next(e for e in st["events"] if e.get("e") == "calib_done")))}


def main() -> None:
    out = HERE.parent / "test" / "vectors"
    out.mkdir(parents=True, exist_ok=True)
    (out / "api_contract.json").write_text(json.dumps(api_contract(), indent=1), encoding="utf-8")
    print("api_contract.json: OK")
    data = {"source": "tworobot.fw.primitives.tof_fit (Python)", "cases": tof_cases()}
    (out / "tof_fit.json").write_text(json.dumps(data), encoding="utf-8")
    print(f"tof_fit.json: {len(data['cases'])} trường hợp, "
          f"{sum(len(c['samples']) for c in data['cases'])} mẫu")


if __name__ == "__main__":
    main()

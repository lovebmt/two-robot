"""Planner: nét vẽ trên lưới → đường đi (wiki/web-ui.md §2, §3; wiki/protocol.md §6).

Toạ độ thế giới: mm, trục x sang phải, y hướng LÊN (đỉnh lưới (i, j) → (i·cell, j·cell),
j tính từ đáy bản đồ). Góc: 0° = +x, dương = ngược chiều kim đồng hồ.
"""
from __future__ import annotations

import itertools
import json
import math
import zlib

_path_ids = itertools.count(1)

HEADING = {(1, 0): 0, (0, 1): 90, (-1, 0): 180, (0, -1): -90}


def _norm(d: int) -> int:
    return (d > 0) - (d < 0)


def _segments_intersect(a, b) -> bool:
    """Hai đoạn ngang/dọc (x0,y0,x1,y1) có điểm chung không."""
    ax0, ax1 = sorted((a[0], a[2]))
    ay0, ay1 = sorted((a[1], a[3]))
    bx0, bx1 = sorted((b[0], b[2]))
    by0, by1 = sorted((b[1], b[3]))
    return ax0 <= bx1 and bx0 <= ax1 and ay0 <= by1 and by0 <= ay1


def _point_seg_dist(px, py, s) -> float:
    x0, y0, x1, y1 = s
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def plan_path(points: list[list[int]], grid: dict, form: dict, shared: dict,
              geom_f: dict | None = None, solo: bool = False) -> dict:
    """points: các đỉnh lưới [[i, j], ...]. Trả {ok, errors, warnings, path}.

    solo=True: chế độ 1 xe — bỏ các ràng buộc của xe sau (khoảng thoát góc, chỗ xuất phát follower).
    """
    cell = float(grid["cell_mm"])
    cols, rows = int(grid["cols"]), int(grid["rows"])
    W, H = cols * cell, rows * cell
    errors: list[str] = []
    warnings: list[str] = []

    pts: list[tuple[int, int]] = []
    for p in points:
        q = (int(p[0]), int(p[1]))
        if not (0 <= q[0] <= cols and 0 <= q[1] <= rows):
            errors.append(f"điểm {q} nằm ngoài bản đồ")
            continue
        if not pts or pts[-1] != q:
            pts.append(q)
    if len(pts) < 2:
        return {"ok": False, "errors": errors + ["cần ít nhất 2 điểm"], "warnings": warnings, "path": None}
    if len(pts) > 2 and pts[-1] in pts[:-1]:
        errors.append("không cho vẽ đường khép kín (điểm cuối quay về điểm đã đi qua)")
        return {"ok": False, "errors": errors, "warnings": warnings, "path": None}

    # Gộp các điểm thẳng hàng thành segment; chỉ cho ngang/dọc, cấm quay đầu 180°.
    segs: list[dict] = []  # {a:(i,j), b:(i,j), dir:(dx,dy), n: số ô}
    for idx, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        if dx != 0 and dy != 0:
            errors.append(f"đoạn {idx + 1} không nằm ngang/dọc theo lưới")
            continue
        d = (_norm(dx), _norm(dy))
        n = abs(dx) + abs(dy)
        if segs and segs[-1]["dir"] == d:
            segs[-1]["b"] = b
            segs[-1]["n"] += n
        elif segs and segs[-1]["dir"] == (-d[0], -d[1]):
            errors.append(f"tại điểm {a}: quay đầu 180° không được phép")
            segs.append({"a": a, "b": b, "dir": d, "n": n})
        else:
            segs.append({"a": a, "b": b, "dir": d, "n": n})
    if errors:
        return {"ok": False, "errors": errors, "warnings": warnings, "path": None}

    out_segs = []
    corners_s = []
    s = 0.0
    for k, sg in enumerate(segs):
        length = sg["n"] * cell
        turn = 0
        if k + 1 < len(segs):
            d0, d1 = sg["dir"], segs[k + 1]["dir"]
            cross = d0[0] * d1[1] - d0[1] * d1[0]
            turn = 90 if cross > 0 else -90
        s += length
        if turn:
            corners_s.append(s)
        out_segs.append({"len": length, "heading_deg": HEADING[sg["dir"]], "turn_after": turn,
                         "a": [sg["a"][0] * cell, sg["a"][1] * cell], "b": [sg["b"][0] * cell, sg["b"][1] * cell]})
    total = s

    # Tự cắt / chạm lại chính nó (bỏ qua 2 đoạn kề nhau, vốn chung 1 đỉnh).
    # Bao gồm cả trường hợp khép kín: điểm cuối nằm trên một đoạn trước đó.
    lines = [(*sg["a"], *sg["b"]) for sg in out_segs]
    for i in range(len(lines)):
        for j in range(i + 2, len(lines)):
            if _segments_intersect(lines[i], lines[j]):
                closed = j == len(lines) - 1 and _point_seg_dist(*lines[j][2:], lines[i]) < 1e-6
                errors.append("không cho vẽ đường khép kín: đoạn cuối chạm lại đoạn " + str(i + 1)
                              if closed else f"đường tự cắt: đoạn {i + 1} và đoạn {j + 1}")

    exit_dist = form["exit_dist"]
    for k in range(1, len(out_segs)):
        if not solo and out_segs[k]["len"] < exit_dist - 1e-6:
            errors.append(f"đoạn {k + 1} dài {out_segs[k]['len']:.0f} mm < khoảng thoát góc {exit_dist:.0f} mm "
                          f"(tăng số ô hoặc kích thước ô)")
    if out_segs[0]["len"] < shared["motion.approach_dist"]:
        warnings.append("đoạn đầu ngắn hơn vùng giảm tốc: leader sẽ không đạt v_cruise")

    # Vị trí follower lúc xuất phát: lùi g_target trên phần kéo dài của đoạn đầu.
    x0, y0 = out_segs[0]["a"]
    h0 = math.radians(out_segs[0]["heading_deg"])
    g = form["g_target"]
    fx, fy = x0 - g * math.cos(h0), y0 - g * math.sin(h0)
    margin = (geom_f or {}).get("rear", 40) + 10
    if not solo and not (-1e-6 + margin <= fx <= W - margin + 1e-6 and margin - 1e-6 <= fy <= H - margin + 1e-6):
        errors.append(f"follower xuất phát ngoài bản đồ ({fx:.0f}, {fy:.0f}) mm: dời điểm đầu vào trong "
                      f"≥ {g:.0f} mm theo hướng ngược đoạn đầu")

    # Vòng quét tại góc chạm đoạn khác?
    rsw = form["r_sweep_leader"] if solo else max(form["r_sweep_leader"], form["r_sweep_follower"])
    for k in range(len(out_segs) - 1):
        cx, cy = out_segs[k]["b"]
        for j, ln in enumerate(lines):
            if j in (k, k + 1):
                continue
            if _point_seg_dist(cx, cy, ln) < rsw + 60:
                warnings.append(f"góc {k + 1}: vòng quét ({rsw:.0f} mm) sát đoạn {j + 1}")

    path = {
        "path_id": next(_path_ids), "units": "mm",
        "start": {"x": x0, "y": y0, "yaw_deg": out_segs[0]["heading_deg"]},
        "segments": out_segs, "corners_s": corners_s, "total_len": total,
        "follower": {"start_s": -g, "start": {"x": fx, "y": fy, "yaw_deg": out_segs[0]["heading_deg"]}},
        "formation": form, "solo": solo, "grid": {"cell_mm": cell, "cols": cols, "rows": rows},
    }
    blob = json.dumps({k: v for k, v in path.items() if k != "path_id"}, sort_keys=True).encode()
    path["crc32"] = f"{zlib.crc32(blob):08x}"
    return {"ok": not errors, "errors": errors, "warnings": warnings, "path": path}

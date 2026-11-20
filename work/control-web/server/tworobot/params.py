"""Tham số động: đọc schema chung, validate, tính giá trị suy ra.

Nguồn chuẩn: work/shared/config.schema.json. Trên ESP32 phần này sẽ được codegen
sang C++ (struct Config + validate + derive) — xem wiki/config-management.md.
Config biểu diễn dạng dict phẳng {"body.length_mm": 160.0, ...}.
"""
from __future__ import annotations

import json
import math
import os
import zlib
from pathlib import Path
from typing import Any

SCHEMA_PATH = Path(
    os.environ.get(
        "TWOROBOT_SCHEMA",
        Path(__file__).resolve().parents[3] / "shared" / "config.schema.json",
    )
)


def load_schema(path: Path = SCHEMA_PATH) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


SCHEMA: dict = load_schema()
PARAMS: dict[str, dict] = {p["key"]: p for p in SCHEMA["params"]}
SHARED_KEYS: list[str] = [k for k, p in PARAMS.items() if p.get("scope") == "shared"]


def defaults() -> dict[str, Any]:
    return {k: p["default"] for k, p in PARAMS.items()}


def shared_defaults() -> dict[str, Any]:
    return {k: PARAMS[k]["default"] for k in SHARED_KEYS}


def coerce(spec: dict, value: Any) -> Any:
    """Ép kiểu + kiểm tra min/max. Lỗi → ValueError (thông báo tiếng Việt)."""
    t = spec["type"]
    if t == "bool":
        if isinstance(value, bool):
            return value
        if value in (0, 1, "0", "1", "true", "false"):
            return value in (1, "1", "true")
        raise ValueError("phải là true/false")
    if isinstance(value, bool):
        raise ValueError("phải là số")
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError("phải là số") from None
    if not math.isfinite(v):
        raise ValueError("phải là số hữu hạn")
    if t == "int":
        v = int(round(v))
    lo, hi = spec.get("min"), spec.get("max")
    if lo is not None and v < lo:
        raise ValueError(f"phải ≥ {lo}")
    if hi is not None and v > hi:
        raise ValueError(f"phải ≤ {hi}")
    return v


def cross_check(c: dict) -> dict[str, str]:
    """Ràng buộc chéo giữa các tham số của 1 xe."""
    errs: dict[str, str] = {}
    if c["range.min_valid_mm"] >= c["range.max_valid_mm"]:
        errs["range.min_valid_mm"] = "phải nhỏ hơn range.max_valid_mm"
    if c["body.axle_to_front_mm"] + c["body.axle_to_rear_mm"] > c["body.length_mm"] + 1.0:
        errs["body.length_mm"] = "phải ≥ (tâm trục→trước) + (tâm trục→sau)"
    if c["range.x_mm"] > c["body.axle_to_front_mm"] + 1.0:
        errs["range.x_mm"] = "mắt cảm biến nằm ngoài mặt trước thân xe"
    if c["range.baseline_mm"] > c["body.width_mm"]:
        errs["range.baseline_mm"] = "khoảng cách 2 mắt lớn hơn bề rộng thân"
    if c["wheel.track_mm"] > c["body.width_mm"] + 60:
        errs["wheel.track_mm"] = "track lớn bất thường so với bề rộng thân"
    return errs


def validate_patch(current: dict, patch: dict) -> tuple[dict, dict[str, str]]:
    """Áp patch lên bản copy. Trả (config mới, lỗi theo khoá). Có lỗi thì không dùng config mới."""
    new = dict(current)
    errors: dict[str, str] = {}
    for k, v in patch.items():
        spec = PARAMS.get(k)
        if spec is None:
            errors[k] = "khoá không tồn tại"
            continue
        try:
            new[k] = coerce(spec, v)
        except ValueError as e:
            errors[k] = str(e)
    if not errors:
        errors.update(cross_check(new))
    return new, errors


def changed_keys(old: dict, new: dict) -> list[str]:
    return [k for k in new if old.get(k) != new[k]]


def changed_groups(old: dict, new: dict) -> set[str]:
    return {PARAMS[k]["group"] for k in changed_keys(old, new) if k in PARAMS}


def idle_keys_changed(old: dict, new: dict) -> list[str]:
    return [k for k in changed_keys(old, new) if PARAMS.get(k, {}).get("apply") == "idle"]


def crc(cfg: dict) -> str:
    blob = json.dumps(cfg, sort_keys=True, separators=(",", ":")).encode()
    return f"{zlib.crc32(blob):08x}"


# ---------------------------------------------------------------- derived ---

NOMINAL_VBAT = 11.1  # 3S danh định, dùng để ước lượng tốc độ tối đa khi lập kế hoạch


def sweep_radius(c: dict) -> float:
    """Bán kính quét khi quay tại chỗ quanh tâm trục bánh (thân hộp)."""
    return math.hypot(max(c["body.axle_to_front_mm"], c["body.axle_to_rear_mm"]), c["body.width_mm"] / 2)


def wheel_v_max(c: dict, vbat: float = NOMINAL_VBAT) -> float:
    """Tốc độ bánh tối đa (mm/s) của bánh yếu hơn ở điện áp pin cho trước."""
    vavail = vbat - c["power.l298n_vdrop_v"]
    vl = (vavail - c["motor.ks_l_v"]) / c["motor.kv_l"]
    vr = (vavail - c["motor.ks_r_v"]) / c["motor.kv_r"]
    return max(0.0, min(vl, vr))


def derive_robot(c: dict) -> dict[str, float]:
    ticks_per_rev = c["enc.ppr_motor"] * 4 * c["enc.gear_ratio"]
    d = c["wheel.diameter_mm"]
    eps = c["wheel.diam_eps"]
    return {
        "ticks_per_rev": ticks_per_rev,
        "mm_per_tick_l": math.pi * d * (1 + eps) / ticks_per_rev,
        "mm_per_tick_r": math.pi * d * (1 - eps) / ticks_per_rev,
        "r_sweep_mm": sweep_radius(c),
        "v_wheel_max_mm_s": wheel_v_max(c),
    }


def formation(cl: dict, cf: dict, shared: dict) -> tuple[dict, dict[str, str], list[str]]:
    """Thông số đội hình từ cấu hình leader (cl), follower (cf) và tham số chung.

    Trả (formation, errors, warnings). Xem wiki/algorithms.md §5 (hình học an toàn).
    """
    errors: dict[str, str] = {}
    warnings: list[str] = []
    margin = shared["corner.margin_mm"]
    rsw_l, rsw_f = sweep_radius(cl), sweep_radius(cf)
    front_f, rear_l = cf["body.axle_to_front_mm"], cl["body.axle_to_rear_mm"]

    # Leader quay tại C: mũi follower phải ngoài vòng quét leader.
    g_hold_min = rsw_l + front_f + margin
    g_hold = max(shared["gap.hold_mm"], g_hold_min)
    # Follower quay tại C: đuôi leader phải ngoài vòng quét follower,
    # đồng thời ToF follower phải đo được leader sau khi quay. Biên 50 mm trên min_valid:
    # sai số tích luỹ (leader dừng, follower tiến theo snapshot) cỡ ±20 mm — sim cho thấy
    # biên 20 mm làm số đo rơi xuống dưới min_valid (ALIGN: NO_TARGET).
    exit_min = max(rsw_f + rear_l + margin, cf["range.x_mm"] + rear_l + cf["range.min_valid_mm"] + 50)
    exit_dist = max(shared["corner.exit_dist_mm"], exit_min)
    gap_min_safe = front_f + rear_l + 15
    g_target = shared["gap.target_mm"]

    if g_target < g_hold + 10:
        errors["gap.target_mm"] = f"phải ≥ {g_hold + 10:.0f} mm (gap chờ an toàn khi leader quay + 10)"
    d_at_target = g_target - cf["range.x_mm"] - rear_l
    if d_at_target > cf["range.max_valid_mm"] - 100:
        errors["gap.target_mm"] = f"ToF sẽ đo {d_at_target:.0f} mm, vượt vùng đo tin cậy"

    v_max_pair = min(wheel_v_max(cl), wheel_v_max(cf))
    if shared["motion.v_cruise"] > 0.7 * v_max_pair:
        warnings.append(
            f"v_cruise {shared['motion.v_cruise']:.0f} mm/s > 70% tốc độ tối đa của xe yếu hơn "
            f"({v_max_pair:.0f} mm/s): follower có thể không đủ dư tốc độ để đuổi"
        )

    form = {
        "g_target": g_target,
        "g_hold": g_hold,
        "g_hold_min": g_hold_min,
        "exit_dist": exit_dist,
        "exit_dist_min": exit_min,
        "gap_min_safe": gap_min_safe,
        "leader_rear_mm": rear_l,
        "r_sweep_leader": rsw_l,
        "r_sweep_follower": rsw_f,
        "v_max_pair": v_max_pair,
    }
    return form, errors, warnings

"""Tính hệ số hiệu chỉnh từ kết quả robot báo về + số đo thực tế người dùng nhập.

Mỗi hàm trả (patch config, thông tin hiển thị). Kiểm tra giới hạn hợp lý để tránh nhập nhầm.
Xem wiki/algorithms.md §6.
"""
from __future__ import annotations

import math


class CalError(ValueError):
    pass


def drive(cfg: dict, res: dict, measured_mm: float) -> tuple[dict, dict]:
    """Đi thẳng: robot báo enc_mm (quãng theo encoder với đường kính hiện tại); người dùng đo thật.

    D_mới = D_cũ × thật / encoder. (Lệch đường kính 2 bánh không tính được từ bài này vì hướng
    được giữ bằng gyro; ảnh hưởng của nó lên quãng đường trung bình triệt tiêu.)
    """
    enc = float(res["enc_mm"])
    if measured_mm <= 0 or enc <= 0:
        raise CalError("quãng đường phải > 0")
    ratio = measured_mm / enc
    if not 0.8 <= ratio <= 1.2:
        raise CalError(f"thật/encoder = {ratio:.3f} ngoài [0.8, 1.2] — kiểm tra lại số đo hoặc PPR/tỉ số truyền")
    d_old = cfg["wheel.diameter_mm"]
    d_new = round(d_old * ratio, 3)
    return {"wheel.diameter_mm": d_new}, {
        "enc_mm": enc, "measured_mm": measured_mm, "ratio": ratio,
        "diameter_old_mm": d_old, "diameter_new_mm": d_new,
        "err_before_pct": (enc - measured_mm) / measured_mm * 100,
    }


def turn(cfg: dict, res: dict, measured_deg: float) -> tuple[dict, dict]:
    """Quay tại chỗ: robot báo est_deg (góc theo gyro đã scale) và cung 2 bánh theo encoder.

    gyro_scale mới = cũ × thật / ước lượng.   track_eff = (cung phải − cung trái) / góc thật.
    """
    est = float(res["est_deg"])
    if abs(measured_deg) < 30:
        raise CalError("nên quay ≥ 90° (tốt nhất 360°) để đo chính xác")
    ratio = measured_deg / est
    if not 0.85 <= ratio <= 1.15:
        raise CalError(f"thật/ước lượng = {ratio:.3f} ngoài [0.85, 1.15] — kiểm tra lại số đo (đã cộng đủ vòng?)")
    s_old = cfg["imu.gyro_scale_z"]
    s_new = round(s_old * ratio, 5)
    if not 0.8 <= s_new <= 1.2:
        raise CalError(f"gyro_scale_z mới {s_new} ngoài [0.8, 1.2]")
    patch = {"imu.gyro_scale_z": s_new}
    info = {"est_deg": est, "measured_deg": measured_deg, "ratio": ratio,
            "gyro_scale_old": s_old, "gyro_scale_new": s_new,
            "err_before_deg_per_90": (est - measured_deg) / abs(measured_deg) * 90}
    arc = float(res["arc_r_mm"]) - float(res["arc_l_mm"])
    t_new = arc / math.radians(measured_deg)
    info.update(track_eff_old_mm=cfg["wheel.track_eff_mm"], track_eff_new_mm=round(t_new, 2))
    if 50 <= t_new <= 400:
        patch["wheel.track_eff_mm"] = round(t_new, 2)
    else:
        info["track_warning"] = "track hiệu dụng tính ra bất thường — bỏ qua"
    return patch, info


def gap(cfg: dict, snap: dict, known_mm: float, keep_diff: bool) -> tuple[dict, dict]:
    """Calib khoảng cách: xe sau đứng yên, mặt mắt ToF cách mặt đuôi xe trước đúng `known_mm`.

    keep_diff=True  (đã Calib ToF quét tường): chỉ dời đều 2 offset → giữ chênh lệch 2 mắt.
    keep_diff=False: đặt riêng từng mắt = known (cần 2 xe thẳng hàng thật tốt, lệch 1° ≈ b·0.017 mm).
    """
    if not 30 <= known_mm <= 1000:
        raise CalError("khoảng cách nên trong 30–1000 mm (khuyến nghị 80–150 mm)")
    dl, dr = float(snap["dl_mm"]), float(snap["dr_mm"])          # đã bù offset hiện tại
    ol, orr = float(snap["offset_l_mm"]), float(snap["offset_r_mm"])
    if keep_diff:
        shift = known_mm - 0.5 * (dl + dr)
        new_l, new_r = ol + shift, orr + shift
    else:
        new_l, new_r = ol + (known_mm - dl), orr + (known_mm - dr)
    for v in (new_l, new_r):
        if not -50 <= v <= 50:
            raise CalError(f"offset tính ra {v:.1f} mm vượt ±50 — kiểm tra khoảng cách đã nhập/vị trí xe")
    return {"range.offset_l_mm": round(new_l, 2), "range.offset_r_mm": round(new_r, 2)}, {
        "known_mm": known_mm, "measured_l_mm": dl, "measured_r_mm": dr,
        "err_before_mm": 0.5 * (dl + dr) - known_mm, "dpsi_deg": snap.get("dpsi_deg"),
        "offset_old": [ol, orr], "offset_new": [round(new_l, 2), round(new_r, 2)], "keep_diff": keep_diff,
    }

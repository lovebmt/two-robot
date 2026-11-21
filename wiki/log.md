# Wiki log

## [2026-11-20] ingest | 2026-11-20-brief
- Tạo: requirements.md, architecture.md, algorithms.md, parameters.md, open-questions.md, index.md
- Tạo ngoài wiki: ops/roadmap.md, ops/decisions.md, README.md

## [2026-11-20] ingest | 2026-11-20-answers-1
- Chốt: web trên PC, liên lạc qua PC (bỏ ESP-NOW), chỉ góc 90°, L298N
- Tạo: protocol.md, web-ui.md, work/hardware/hardware-intake.md
- Viết lại: requirements.md, architecture.md, algorithms.md (PC coordinator, ToF snapshot/align), parameters.md, open-questions.md

## [2026-11-20] ingest | 2026-11-20-answers-2
- Chốt: ToF200C (VL53L0X), có MPU6050, GA25-370, ESP32 DevKit
- Cập nhật: requirements.md (C3, C5, N3, N5), architecture.md (ToF200C, IMU, I²C, GPIO), algorithms.md (fusion gyro, quay vòng kín theo gyro), parameters.md (imu.*), open-questions.md, hardware-intake.md
- Cập nhật ngoài wiki: ops/decisions.md (D8–D13), ops/roadmap.md, README.md

## [2026-11-20] ingest | 2026-11-20-answers-3
- Chốt: VL53L0X, motor 12 V (3S), sim toàn phần + view chung, cấu hình động lưu flash
- Tạo: motor-control.md, simulation.md, config-management.md
- Cập nhật: requirements.md (C5–C7, F23–F29, N9–N10), architecture.md (cấu trúc lib, 12 V), algorithms.md, protocol.md, parameters.md (hw.*, motor LUT/adapt), web-ui.md, open-questions.md, index.md
- Cập nhật ngoài wiki: ops/roadmap.md (sim-first), ops/decisions.md (D14–D18), hardware-intake.md, README.md

## [2026-11-20] ingest | 2026-11-20-answers-4
- Chốt: GPIO cố định trong code (pins.h). Kích thước/khoảng cách/hệ số là tham số động, firmware xử lý lúc chạy
- Viết lại: config-management.md (thêm §3 firmware xử lý tham số động: ParamService, Derived, swap ở ranh giới chu kỳ, onParams bumpless, live/idle)
- Cập nhật: parameters.md (bỏ hw.pins, thêm power.*), requirements.md (C7, F29, F30), architecture.md, web-ui.md, open-questions.md
- Cập nhật ngoài wiki: ops/decisions.md (D17 sửa, D19), ops/roadmap.md, hardware-intake.md, README.md

## [2026-11-20] ingest | 2026-11-20-answers-5
- Bắt đầu code: Web Control + sim Python (work/control-web), schema work/shared/config.schema.json
- Cập nhật: simulation.md (trạng thái cài đặt Python)
- Cập nhật ngoài wiki: ops/decisions.md (D20–D23), ops/roadmap.md (tiến độ thực tế)

## [2026-11-20] ingest | 2026-11-20-answers-6
- Code: mô hình trôi MPU6050 (sim), ImuCalibrator + ZUPT + bù nhiệt (fw), bước 0 calib IMU (coordinator), UI IMU
- Cập nhật: algorithms.md (§2.1 trôi MPU6050 và calib, bảng kết quả sim), simulation.md, parameters.md (imu.*)
- Cập nhật ngoài wiki: ops/decisions.md (D24), ops/roadmap.md

## [2026-11-20] ingest | 2026-11-20-answers-7
- Chốt: robot thật giao tiếp HTTP qua LAN; không cho vẽ đường khép kín
- Viết lại: protocol.md (HTTP API, polling /api/state, hộp thư sự kiện, heartbeat bằng request)
- Cập nhật: architecture.md, requirements.md (F2, F7), web-ui.md, config-management.md, open-questions.md (Q12, Q25)
- Cập nhật ngoài wiki: ops/decisions.md (D25, D26; D8 bị thay), ops/roadmap.md

## [2026-11-20] ingest | yêu cầu viết firmware ESP32
- Code: work/firmware (lib/core C++, HAL ESP32, net, main, test native, tools/gen_vectors.py), work/shared/codegen_cpp.py; schema thêm invert/pwm/vbat_divider
- Cập nhật ngoài wiki: ops/decisions.md (D29), ops/roadmap.md, work/firmware/README.md

## [2026-11-20] query | Đặt xe lệch lúc đầu; kiểm tra trước khi chạy; wizard hiệu chuẩn
- Code: preflight (coordinator), CAL_DRIVE/CAL_TURN, SNAPSHOT chạy từ IDLE, calibration.py, API /api/run/check + /api/robots/{id}/cal/*, tab Hiệu chuẩn, đặt xe lệch trong sim
- Cập nhật: algorithms.md (§5.1 kèm bảng ảnh hưởng đặt lệch, §6 wizard), web-ui.md, parameters (start.*)
- Cập nhật ngoài wiki: ops/decisions.md (D28), ops/roadmap.md

## [2026-11-20] query | ToF lắp không thẳng hàng; nhiễu IMU + lệch bánh
- Code: sim sai lệch lắp ToF, TOF_CAL + endpoint/UI Calib ToF, thanh trượt nhiễu gyro / đường kính bánh, mã IMU_NOISY
- Cập nhật: algorithms.md (§6.1 kèm bảng kết quả), simulation.md
- Cập nhật ngoài wiki: ops/decisions.md (D27), ops/roadmap.md

## [2026-11-21] query | Chế độ 1 xe chạy theo đường vẽ sẵn
- Code: planner `solo`, coordinator `_run_solo` + `preflight_solo`, API `mode` cho /api/plan, /api/run/start, /api/run/check; UI chọn 2 xe/1 xe (tab Bản đồ + Chạy); sim batch `--solo`; test planner + sim
- Sửa kèm: firmware 0.2.6 — LINK_LOST giả (trừ không dấu khi heartbeat mới hơn `now`), log chẩn đoán WiFi/heartbeat/reset; `safety.link_timeout_ms` 500 → 1500
- Cập nhật ngoài wiki: ops/decisions.md (D30)

## [2026-11-21] query | Hiệu chuẩn thực tế xe 198a2c, ma sát tĩnh, chế độ bám tấm
- Code: ToF lọc theo range status + tín hiệu (`range.min_signal_mcps` mặc định 1.5); CAL_KS + kbreak + tăng điện áp khi kẹt + ω quay tối thiểu + chấp nhận dừng khi kẹt (fw 0.2.8); cờ pause kẹt khi IDLE (fw 0.2.9); `FOLLOW_PLATE` + API /api/follow/start|stop + UI "1 xe bám tấm" (fw 0.3.0)
- Phần cứng: GA25-370 12V 280RPM → hộp số ≈ 1:20.24 (không phải 1:34); thân 199×199, tâm trục → mặt sau 68 mm
- Cập nhật ngoài wiki: ops/decisions.md (D31), work/hardware (robot-198a2c rev 22, hardware-intake.md)

## [2026-11-21] query | Luồng tham số RAM ↔ flash ↔ Web Control
- Cập nhật: config-management.md (thêm §7 luồng thực tế đối chiếu code, §7.1 hai loại tham số + chỗ dễ nhầm tham số chung, §7.2 primitive chép tham số; gạch ý "web hỏi chiều đồng bộ" ở §5 vì chưa làm)
- Cập nhật: index.md (mô tả config-management)

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

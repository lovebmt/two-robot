# Roadmap

Nguyên tắc: **sim trước, xe thật sau**. View được dựng trên sim, nên khi xe thật cắm vào thì hiển thị được ngay mà không phải làm lại.
Thứ tự: M1 khung → M2 lõi + sim batch → M3 server + view trên sim → (song song M4a phần cứng) → M4 firmware 1 xe → M5 → M6.

## M0 — Kế hoạch và repo ✅ (2026-11-20)
- [x] Yêu cầu, kiến trúc, thuật toán, giao thức, UI, tham số
- [x] Motor mạnh yếu và ma sát ([motor-control](../wiki/motor-control.md)), sim và view ([simulation](../wiki/simulation.md)), cấu hình động ([config-management](../wiki/config-management.md))
- [ ] Chốt Q15 (tỉ số hộp số), Q16, Q21, Q22 trước M4. Kích thước không chặn vì nhập trên web

## Tiến độ thực tế (2026-11-20): làm M2/M3 bằng Python trước (D20)
- [x] `work/shared/config.schema.json`: 60+ tham số, scope robot/shared, apply live/idle
- [x] Firmware ảo Python (`tworobot/fw`): motor control L1/L3/L4, fusion gyro + ZUPT, KF gap, 7 primitive, ParamService, máy trạng thái, an toàn
- [x] Sim (`tworobot/sim`): motor lệch, L298N, ma sát theo vùng, trượt, gyro bias, ToF ray-cast 5 tia, va chạm, gây lỗi; batch CLI
- [x] Server FastAPI: registry, SimLink, relay, coordinator 10 bước, planner/validator, config API, ghi log, replay, nghe beacon
- [x] Web UI: Thiết bị, Cấu hình (form tự sinh), Bản đồ (vẽ lưới ràng buộc 90°), Chạy/Xem (view chung, đồ thị, bảng bước, gây lỗi, vật lý)
- [x] 16 test pytest; batch 36/36 kịch bản (L/rect/zigzag × 4 seed × 3 mức sai lệch) xong, 0 va chạm, gap RMSE 4–9 mm, sai số điểm cua ≤ 27 mm
- [x] (đợt 6) Mô hình trôi MPU6050 trong sim; calib gyro khi đứng yên + ZUPT + bù nhiệt; bước 0 calib IMU trong coordinator; UI hiển thị IMU, nút Calib IMU, Lưu k nhiệt
- [x] (đợt 7) NetLink HTTP (poll 20 Hz, hộp thư sự kiện, ack đồng bộ), quét HTTP, robot HTTP ảo `tworobot-vrobot`, test end-to-end qua socket thật; cấm đường khép kín
- [ ] Wizard hiệu chuẩn scale gyro (quay 360° có mốc ngoài) trên web
- [ ] Firmware ESP32 (M4): HTTP server theo protocol.md, port `fw/` sang C++
- [ ] Vẽ vùng ma sát trên UI; replay có kéo timeline
- [ ] Unit test riêng cho từng primitive; kịch bản đổi tham số giữa lúc chạy

## M1 — Khung dự án và schema
- [ ] `work/firmware`: PlatformIO env `esp32dev` và `native`, cấu trúc `lib/{core,app,proto,hal_api}`, Unity test
- [ ] `work/sim`: CMake dùng lại `lib/*`, thêm `hal_sim/`, `world/`, deps (ArduinoJson, cpp-httplib, IXWebSocket)
- [ ] `work/shared`: `config.schema.json`, `telemetry.schema.json`, mô tả struct UDP. **Codegen** ra C++ / Python / TS
- [ ] `work/control-web/server` (FastAPI) và `work/control-web/web` (Vite + TS), có skeleton
- [ ] `make test` (pio native, ctest, pytest, npm build)

**Xong khi:** `make test` xanh. Codegen sinh được `Config` cho cả 3 ngôn ngữ.

## M2 — Lõi thuật toán và sim batch (chưa cần UI)
- [ ] `lib/core`: kinematics, profile, dừng tại điểm, **motor control L1–L7** (LUT FF, PI, bão hoà, thích nghi, kick tốc độ thấp)
- [ ] fusion gyro + encoder, quay vòng kín theo gyro, mô hình 2 ToF, KF gap, luật FOLLOW
- [ ] `lib/app`: primitive (DRIVE_TO, TURN, FOLLOW, SNAPSHOT, REACQUIRE, ALIGN, HOLD), safety
- [ ] `lib/app/params`: ParamService (patch → validate → Derived → pending → swap ở đầu chu kỳ), `onParams(ChangeSet)` bumpless, live/idle
- [ ] Unit test và kịch bản sim **đổi tham số giữa lúc chạy** (gain, gap.target): không giật, không lẫn snapshot
- [ ] `world`: motor DC lệch từng bánh, L298N và pin, ma sát theo vùng, trượt, caster, encoder, gyro, ToF ray-cast, va chạm
- [ ] Scheduler deterministic (bước 1 ms, task đúng tần số), seed
- [ ] `sim run` batch: kịch bản YAML, metric, pass/fail. Ở mốc này coordinator góc cua viết tạm bằng script trong sim

**Xong khi:** FOLLOW trên đoạn thẳng và 1 góc cua đạt N1–N3 trong batch, kể cả biến thể N9 (lệch motor ±20 %, ma sát ±30 %).

## M3 — Server và view, chạy trên robot ảo
- [ ] `lib/proto` cùng transport sim (HTTP, WS, UDP trên localhost, beacon). `sim serve` interactive, có kênh truth
- [ ] Server: discovery, robots, relay, timesync, heartbeat, E-STOP, **coordinator góc cua** (thay script tạm ở M2), logger JSONL
- [ ] Config: API đầy đủ, form tự sinh, Apply/Save/Diff. "Flash" ảo của robot ảo là file
- [ ] Map editor ràng buộc 90°, planner, validator
- [ ] **View chung**: canvas các lớp, corner stepper, đồ thị, điều khiển sim, nút gây lỗi, replay
- [ ] Batch nhiều seed gọi được từ UI

**Xong khi:** quy trình đầy đủ trên 2 robot ảo (dò → gán vai trò → config → vẽ → chạy → replay) qua kịch bản chữ L, chữ nhật, zig-zag, kèm mất gói 10 % và lệch motor.

## M4a — Phần cứng (song song M2/M3)
- [ ] Đi dây: 3S, buck 5 V, GND chung, tháo jumper ENA/ENB, kiểm tra mức điện áp encoder. Gắn tấm phản xạ, gắn MPU6050 có đệm
- [ ] Ghi số đo vào [intake](../work/hardware/hardware-intake.md). Chép GPIO vào `src/board/pins.h`

## M4 — Firmware HAL ESP32 (1 xe)
- [ ] `hal_esp32`: LEDC, PCNT (có timestamp cạnh xung), 2× VL53L0X (XSHUT), MPU6050 (bus 2), ADC pin, LittleFS
- [ ] WiFi + AP portal, transport thật cho `lib/proto`. ParamService trên FreeRTOS, LittleFS save/load
- [ ] Wizard hiệu chuẩn: chiều motor, quét PWM, step response, đi thẳng, quay, ToF. **Xuất mô hình motor sang sim**
- [ ] Xe thật hiện lên **view chung** (badge REAL)

**Xong khi:** đi thẳng 1 m sai ≤ 1 cm, quay 90° sai ≤ 1.5°. Sim chạy với mô hình motor của xe thật cho kết quả gần xe thật.

## M5 — 1 xe theo đường
- [ ] Leader chạy chữ L và chữ nhật bằng coordinator. So log thật với sim cùng kịch bản (replay cạnh nhau)

## M6 — 2 xe thật bám nhau
- [ ] FOLLOW trên đoạn thẳng, tinh chỉnh live bằng slider rồi Save flash
- [ ] Góc cua 10 bước. Kịch bản lỗi: rút router, che ToF, đẩy xe, pin yếu

**Xong khi:** đạt [tiêu chí nghiệm thu](../wiki/requirements.md#4-tiêu-chí-nghiệm-thu-v1).

## M7 — Mở rộng (tuỳ chọn)
Chế độ MIXED (leader thật + follower ảo), chạy song song các bước ở góc cua, đường khép kín, OTA firmware.

## Rủi ro chính
| Rủi ro | Giảm thiểu |
|---|---|
| Sim khác xe thật (sim-to-real gap) | Mô hình motor lấy từ step response của xe thật, ToF mô phỏng bằng nhiều tia, so replay sim với thật ở M5 |
| ToF200C FOV 25° thấy cả sàn hoặc vật bên cạnh | Tấm phản xạ rộng và cao, gating, thử góc toe-in (thử trước trong sim) |
| Motor lệch nhau, L298N phi tuyến, pin tụt | LUT riêng từng bánh, bù theo V_bat, PI, thích nghi, bão hoà ưu tiên ω |
| Tốc độ thấp bị giật (ma sát tĩnh) | Đo chu kỳ xung, điều khiển vị trí + kick, chọn tần số PWM |
| Rung motor làm nhiễu gyro | Đệm mút, DLPF, ZUPT |
| WiFi trễ hoặc mất gói | Vòng nhanh nằm trên xe, UDP stream, WS có ACK, timeout an toàn |
| Đổi tham số khi đang chạy làm xe giật hoặc mất ổn định | Snapshot bất biến mỗi chu kỳ, bumpless, khoá `idle`, validate ở firmware, test trong sim trước |

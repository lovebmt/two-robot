# Roadmap

Nguyên tắc: **làm lõi thuật toán và sim trước, phần cứng sau**. Mỗi mốc có tiêu chí xong rõ ràng. Mỗi task là 1 hoặc vài commit nhỏ.
Có thể làm song song: M2/M3 (phần mềm) chạy song song với M4a (lắp và đo phần cứng).

## M0 — Kế hoạch và repo ✅ (2026-11-20)
- [x] Brief, yêu cầu, kiến trúc, thuật toán, thông số, giao thức, UI
- [x] Chốt Q1, Q3, Q4, Q5, Q7, Q8, Q11, Q19
- [ ] Chốt Q15–Q18, Q21, Q22 và bảng GPIO (trước M4)

## M1 — Khung dự án
- [ ] `work/firmware`: PlatformIO, env `esp32dev` và `native`, `lib/core` rỗng, Unity test chạy được
- [ ] `work/control-web/server`: FastAPI skeleton, cấu trúc module (discovery, robots, relay, coordinator, planner, validator, timesync, logger)
- [ ] `work/control-web/web`: Vite + TS skeleton, khung 6 màn hình
- [ ] `work/sim`: CLI native dùng `lib/core`
- [ ] `work/shared/schemas`: JSON Schema cho config, path, message. Struct nhị phân UDP mô tả 1 nơi, sinh ra code C và Python
- [ ] Script: `make test` chạy `pio test -e native`, `pytest`, `npm run build`

**Xong khi:** `make test` xanh.

## M2 — Lõi thuật toán (C++ thuần) và sim
- [ ] Kinematics, profile hình thang, dừng tại điểm
- [ ] Fusion hướng gyro + encoder, giữ hướng khi đi thẳng, quay 90° vòng kín theo gyro
- [ ] Mô hình đo 2 ToF (`ρ`, `Δψ`, mép tấm), KF gap, gating
- [ ] Luật FOLLOW, các primitive: DRIVE_TO, TURN, FOLLOW, SNAPSHOT, REACQUIRE, ALIGN, HOLD
- [ ] Sim vật lý 2 xe: motor + vùng chết, trượt, nhiễu encoder, gyro bias, ToF (FOV/nhiễu/mép tấm)
- [ ] "Robot ảo": sim nói đúng giao thức (UDP/WS/HTTP) để server test được mà không cần xe thật

**Xong khi:** unit test xanh. FOLLOW trên đoạn thẳng trong sim đạt N1/N2.

## M3 — Web Control
- [ ] Discovery (beacon, mDNS, scan). Màn Devices, Identify, gán vai trò
- [ ] Config, hình xem trước xe, giá trị suy ra, push/pull/diff
- [ ] Grid editor có ràng buộc 90°, planner, validator
- [ ] Relay UDP, time sync, heartbeat, E-STOP
- [ ] **Corner coordinator** (máy trạng thái 10 bước, timeout, retry/abort)
- [ ] Màn Simulate và Run (corner stepper, đồ thị, log)

**Xong khi:** 2 robot ảo chạy trọn kịch bản chữ L, chữ nhật, zig-zag, không va chạm. Có thêm kịch bản mất relay và mất gói 10 %.

## M4a — Phần cứng: lắp, đo, bàn giao (song song M2/M3)
- [ ] Điền [hardware-intake](../work/hardware/hardware-intake.md) và chốt GPIO
- [ ] Đi dây: buck 5 V, GND chung, tháo jumper ENA/ENB, kiểm tra mức điện áp encoder
- [ ] Gắn tấm phản xạ đuôi. Gắn MPU6050 có đệm chống rung

## M4 — Firmware HAL và mạng (1 xe)
- [ ] MotorL298N + EncoderPCNT, PI bánh, bài quét vùng chết
- [ ] 2× ToF200C (XSHUT đổi địa chỉ, continuous), MPU6050 (bias, 200 Hz)
- [ ] WiFi + AP portal, beacon, mDNS, HTTP, WS, UDP, time sync, NVS config
- [ ] Safety: heartbeat, E-STOP, watchdog, pin yếu, stall
- [ ] Các bài hiệu chuẩn (F14) gọi được từ web

**Xong khi:** 1 xe chạy thẳng 1 m sai ≤ 1 cm và quay 90° sai ≤ 1.5°, điều khiển từ web.

## M5 — 1 xe theo đường
- [ ] Leader chạy chữ L và chữ nhật bằng primitive từ coordinator. Telemetry khớp sim

## M6 — 2 xe bám nhau
- [ ] Relay và FOLLOW trên đoạn thẳng thật, tinh chỉnh KF và hệ số
- [ ] Góc cua 10 bước trên xe thật
- [ ] Kịch bản lỗi: rút router, che ToF, đẩy leader lệch, pin yếu

**Xong khi:** đạt [tiêu chí nghiệm thu](../wiki/requirements.md#4-tiêu-chí-nghiệm-thu-v1).

## M7 — Tối ưu (tuỳ chọn)
Chạy song song các bước ở góc cua, tăng tốc, đường khép kín nhiều vòng, sim WASM trong browser, OTA firmware.

## Rủi ro chính
| Rủi ro | Giảm thiểu |
|---|---|
| ToF200C FOV 25° thấy cả sàn hoặc vật bên cạnh | Tấm phản xạ rộng và đủ cao, gắn mắt ToF cao hơn sàn, gating, thử góc toe-in |
| L298N sụt áp và vùng chết lớn làm chạy chậm không mượt | Feedforward + bù vùng chết, pin đủ áp, điều khiển vị trí khi dừng |
| Rung motor làm nhiễu gyro | Đệm mút, DLPF 42 Hz, ZUPT |
| Trượt bánh khi quay | Quay vòng kín theo gyro, gia tốc thấp, ToF ALIGN cho follower |
| WiFi trễ hoặc mất gói (qua PC) | Vòng nhanh nằm trên xe, UDP cho stream, WS có ACK cho lệnh, timeout an toàn |
| Router bật client isolation | Router riêng, tài liệu hướng dẫn cài đặt |

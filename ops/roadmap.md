# Roadmap

Nguyên tắc: **làm lõi thuật toán và sim trước, phần cứng sau**. Mỗi mốc có tiêu chí xong rõ ràng. Mỗi task nên là 1 hoặc vài commit nhỏ.

## M0 — Kế hoạch và repo ✅ (2026-11-20)
- [x] Brief, yêu cầu, kiến trúc, thuật toán, thông số, câu hỏi mở
- [ ] Chốt câu hỏi mở Q1–Q14

## M1 — Khung dự án
- [ ] `work/firmware`: PlatformIO, các env `esp32` và `native`, `lib/core` rỗng, Unity test chạy được
- [ ] `work/control-web`: FastAPI skeleton, Vite + TS skeleton, script dev
- [ ] `work/sim`: CLI native dùng `lib/core`
- [ ] Schema JSON cho config và path (`work/control-web/schemas/`) dùng chung cho validator
- [ ] CI cục bộ: `pio test -e native`, `pytest`, `npm run build`

**Xong khi:** 3 lệnh build/test đều xanh.

## M2 — Lõi thuật toán (C++ thuần) và mô phỏng
- [ ] Kinematics, odometry, fusion góc
- [ ] Path: tham số hoá `s`, chiếu pose, Stanley
- [ ] Profile vận tốc có dừng tại góc
- [ ] KF gap và gating
- [ ] Máy trạng thái leader/follower và giao thức sự kiện góc cua
- [ ] Sim 2 xe (nhiễu, trễ, mất gói, mô hình ToF) xuất JSON
- [ ] Bộ kịch bản: thẳng 3 m, chữ L, chữ nhật, zig-zag, các góc sát nhau

**Xong khi:** sim đạt N1–N3 và không có va chạm ở tất cả kịch bản.

## M3 — Web Control
- [ ] Discovery: mDNS, UDP, subnet scan. Màn Devices, identify, gán vai trò
- [ ] Màn Config theo [parameters.md](../wiki/parameters.md), lưu/mở JSON
- [ ] Grid editor: vẽ, sửa, xoá, đặt start, hiển thị vị trí follower
- [ ] Planner (nét vẽ → ops) và validator (F9)
- [ ] Xem sim trong browser (dùng JSON từ sim CLI, sau này dùng WASM)
- [ ] Màn Run/Monitor: lệnh, telemetry realtime, đồ thị, ghi log

**Xong khi:** dùng "xe giả" (sim chạy như robot ảo qua WS) đi trọn quy trình dò → cấu hình → vẽ → chạy.

## M4 — Firmware HAL và mạng (1 xe)
- [ ] Motor + PCNT encoder, PI bánh, bài quét `ks`/`kv`
- [ ] 2 ToF (XSHUT, continuous), IMU, bias gyro
- [ ] WiFi + captive portal, mDNS, UDP beacon, REST, WS, NVS config
- [ ] Safety: heartbeat timeout, E-STOP, watchdog, pin yếu
- [ ] Các bài hiệu chuẩn (F14)

**Xong khi:** 1 xe chạy thẳng 1 m sai ≤ 1 cm và quay 90° sai ≤ 2°.

## M5 — 1 xe theo đường
- [ ] Nạp đường từ web, chạy chữ L/chữ nhật, telemetry khớp sim

## M6 — 2 xe bám nhau
- [ ] ESP-NOW `PeerState` + sự kiện có ACK
- [ ] Follower bám trên đoạn thẳng (KF + ToF), tinh chỉnh
- [ ] Giao thức qua góc cua stop-and-go
- [ ] Kịch bản lỗi: rút WiFi, che cảm biến, đẩy leader lệch

**Xong khi:** đạt tiêu chí nghiệm thu ở [requirements §4](../wiki/requirements.md#4-tiêu-chí-nghiệm-thu-v1).

## M7 — Mở rộng (tuỳ chọn)
Góc 45° và cua cung tròn ở tốc độ thấp, đường khép kín nhiều vòng, sim WASM trong browser, OTA firmware, nhiều hơn 2 xe.

## Rủi ro chính
| Rủi ro | Giảm thiểu |
|---|---|
| ToF đo nhiễu hoặc mất khi leader quay hay lệch góc | Tấm phản xạ phẳng, gating KF, tắt ToF trong lúc cua, có radio làm nguồn chính khi đó |
| Drift odometry làm 2 xe lệch điểm cua | Gyro, hiệu chuẩn, đoạn thẳng ngắn. Sau này: mốc vạch line/AprilTag |
| Trượt bánh khi quay tại chỗ | Quay vòng kín theo gyro, giới hạn gia tốc góc |
| WiFi trễ hoặc mất gói | Điều khiển realtime nằm trên xe, ESP-NOW cho link peer, timeout an toàn |
| ESP-NOW và WiFi STA khác kênh | Ép ESP-NOW dùng kênh của AP (đọc kênh sau khi kết nối) |

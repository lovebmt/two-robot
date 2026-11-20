# Nhật ký quyết định

Định dạng: `D# — ngày — quyết định — lý do — trạng thái`

- **D1 — 2026-11-20 — Web Control chạy trên PC, xe chỉ có API.** Lý do: UI lưới, sim, log quá nặng cho ESP32, và browser cần server để dò LAN. **Chốt** (Q1).
- **D2 — 2026-11-20 — Firmware dùng PlatformIO + Arduino-ESP32 + FreeRTOS. Lõi thuật toán viết C++ thuần trong `lib/core`.** Lý do: nhanh có kết quả, test được trên native. *Đề xuất*.
- **D3 — 2026-11-20 — Bám xe theo độ dài cung `s` trên đường chung. ToF hiệu chỉnh gap qua Kalman filter.** Lý do: 2 cảm biến song song không đo được lệch ngang, nên dùng đường biết trước làm khung. *Đề xuất*.
- **D4 — 2026-11-20 — Sim dùng chung `lib/core`, build native trước, WASM sau.** *Đề xuất*.
- ~~D5 — Link ESP-NOW~~ → **Huỷ** 2026-11-20, thay bằng D8.
- **D6 — 2026-11-20 — Server PC dùng Python FastAPI, frontend dùng Vite + TS + Canvas.** *Đề xuất* (Q13).
- **D7 — 2026-11-20 — Chỉ có góc 90° và quay tại chỗ theo stop-and-go.** **Chốt** (Q7/Q8).
- **D8 — 2026-11-20 — Mọi liên lạc giữa 2 xe đi qua PC: UDP cho state stream và relay, WS cho lệnh/sự kiện có ACK, có time sync.** Lý do: yêu cầu người dùng. UDP tránh head-of-line blocking của TCP. **Chốt**.
- **D9 — 2026-11-20 — Phân chia: bám trên đoạn thẳng chạy tự chủ trên follower (ToF + encoder + gyro). Góc cua do PC điều phối theo primitive.** Lý do: vòng nhanh không được phụ thuộc độ trễ WiFi. Góc cua chậm và rời rạc nên chịu được trễ, đồng thời dễ debug và hiển thị trên UI. **Chốt**.
- **D10 — 2026-11-20 — Neo khoảng cách tới góc cua bằng ToF snapshot khi cả 2 xe đứng yên, căn song song (ALIGN) bằng ToF sau khi follower quay.** Lý do: follower quay đúng chỗ leader quay, không phụ thuộc drift odometry. **Chốt**.
- **D11 — 2026-11-20 — L298N: PWM ~2 kHz, feedforward + bù vùng chết, phanh nhanh khi dừng, buck 5 V riêng cho ESP32.** *Đề xuất*.
- **D12 — 2026-11-20 — Hướng = fusion gyro MPU6050 (chính) + encoder. Quay 90° vòng kín theo gyro với góc mục tiêu tuyệt đối. ZUPT bias mỗi lần dừng.** Lý do: phần cứng có MPU6050 (đợt 2). **Chốt**.
- **D13 — 2026-11-20 — 2× ToF200C trên I²C bus 0, đổi địa chỉ bằng XSHUT. MPU6050 trên I²C bus 1.** *Đề xuất*, chờ GPIO thực tế.

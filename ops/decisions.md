# Nhật ký quyết định

Định dạng: `D# — ngày — quyết định — lý do — trạng thái`

- **D1 — 2026-11-20 — Web Control chạy trên PC, xe chỉ có API.** Lý do: UI lưới, sim, log quá nặng cho ESP32, và browser cần server để dò LAN. *Đề xuất*, chờ Q1.
- **D2 — 2026-11-20 — Firmware dùng PlatformIO + Arduino-ESP32 + FreeRTOS. Lõi thuật toán viết C++ thuần trong `lib/core`.** Lý do: nhanh có kết quả, nhiều thư viện ToF/IMU, test được trên native. *Đề xuất*.
- **D3 — 2026-11-20 — Bám xe theo độ dài cung `s` trên đường chung. ToF và radio chỉ hiệu chỉnh gap qua Kalman filter.** Lý do: 2 cảm biến song song không đo được lệch ngang, nên dùng đường biết trước làm khung. *Đề xuất*.
- **D4 — 2026-11-20 — Sim dùng chung `lib/core`, build native trước, WASM sau.** Lý do: thuật toán trong sim và trên xe là một, không bị lệch. *Đề xuất*.
- **D5 — 2026-11-20 — Link leader→follower dùng ESP-NOW, PC relay làm fallback.** Lý do: độ trễ thấp, không phụ thuộc router. *Đề xuất*, chờ Q3.
- **D6 — 2026-11-20 — Server PC dùng Python FastAPI, frontend dùng Vite + TS + Canvas.** *Đề xuất*, chờ Q13.
- **D7 — 2026-11-20 — v1 chỉ có góc 90° và quay tại chỗ theo giao thức stop-and-go có bắt tay sự kiện.** *Đề xuất*, chờ Q7/Q8.

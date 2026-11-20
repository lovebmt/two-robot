# Câu hỏi mở (cần chốt)

Mỗi câu có **giả định mặc định**. Tài liệu đang thiết kế theo giả định này cho tới khi bạn trả lời khác.

| # | Câu hỏi | Giả định mặc định |
|---|---|---|
| Q1 | Brief có nhắc "web server của 1 con". Web có cần chạy trên ESP32 không, hay chỉ trên PC? | Web chính chạy trên PC. Mỗi xe chỉ có REST/WS API, thêm 1 trang status/cấu hình WiFi rất nhỏ |
| Q2 | Xe nhận WiFi bằng cách nào (hard-code, captive portal, hay qua USB serial)? | Captive portal (AP mode) khi chưa có WiFi. Đổi được qua web |
| Q3 | Có cho phép 2 xe nói chuyện trực tiếp (ESP-NOW) không, hay mọi thứ đi qua PC? | Có ESP-NOW, PC relay làm fallback |
| Q4 | Loại cảm biến khoảng cách định dùng (ToF VL53L0X/VL53L1X, siêu âm, IR Sharp)? | VL53L1X |
| Q5 | Có gắn IMU (gyro) được không? | Có. Rất nên có để quay 90° chính xác |
| Q6 | 2 cảm biến song song hay nghiêng ra ngoài? Mặt sau leader có tấm phản xạ phẳng không? | Song song, có tấm phẳng sáng màu ở đuôi |
| Q7 | Đường chỉ có góc 90° hay cả 45° và cung tròn? | v1: chỉ 90° (vẽ theo cạnh lưới) |
| Q8 | Khi cua, cả 2 xe luôn quay tại chỗ (stop-and-go), hay cho phép cua cung tròn ở tốc độ thấp? | Luôn quay tại chỗ (v1) |
| Q9 | Gap định nghĩa là tâm–tâm, hay từ mũi follower tới đuôi leader? | Lưu nội bộ tâm–tâm. UI cho nhập kiểu "khe hở" và tự quy đổi |
| Q10 | Tốc độ mong muốn, kích thước sân, cỡ ô lưới? | 200 mm/s, sân 3×2 m, ô 100 mm |
| Q11 | Phần cứng đã có sẵn chưa (model động cơ/encoder/driver, ESP32 loại nào)? | Chưa. Dùng BOM đề xuất |
| Q12 | Đường có cần khép kín và lặp nhiều vòng không? | Có hỗ trợ khép kín, mặc định chạy 1 lần |
| Q13 | Ngôn ngữ server PC: Python hay Node.js? | Python FastAPI |
| Q14 | Có cần chạy chế độ "vai trò đảo" (đuôi thành đầu, chạy lùi đường) không? | Không ở v1 |

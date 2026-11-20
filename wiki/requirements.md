# Yêu cầu

Nguồn: [brief gốc](../raw/2026-11-20-brief.md). Các giả định chưa chốt được đánh dấu **[GĐ]** và liệt kê ở [open-questions.md](open-questions.md).

## 1. Phạm vi

| Thành phần | Mô tả |
|---|---|
| Robot ×2 | ESP32, thân hình hộp, 3 bánh: 2 bánh sau dẫn động có encoder, 1 bánh caster nhỏ phía trước. Có 2 cảm biến khoảng cách gắn ở đầu xe |
| Web Control | Chạy trên PC trong cùng mạng LAN. Dò tìm xe, cấu hình, vẽ đường, ra lệnh, giám sát |
| Mạng | Cả 2 ESP32 vào cùng WiFi (STA). Có thêm link radio trực tiếp giữa 2 xe (ESP-NOW) **[GĐ]** |

Phần ngoài phạm vi v1: tránh vật cản lạ, định vị tuyệt đối (camera/UWB), nhiều hơn 2 xe, đường cong tự do.

## 2. Yêu cầu chức năng

### 2.1 Dò tìm và kết nối
- **F1** Mỗi ESP32 tự vào WiFi đã cấu hình. Lần đầu (chưa có WiFi) xe phát AP + captive portal để nhập SSID/password **[GĐ]**.
- **F2** Web Control tự dò IP các xe trong LAN bằng 3 cách: mDNS (`_tworobot._tcp`), UDP beacon broadcast, và quét HTTP `/api/info` trên subnet nếu 2 cách trên không chạy.
- **F3** Danh sách xe hiển thị: tên, MAC, IP, firmware version, pin, RSSI, trạng thái. Có nút "nháy LED/kêu bíp" để nhận diện xe thật.
- **F4** Người dùng gán vai trò **LEADER/FOLLOWER**. Vai trò được lưu trong NVS của xe và đổi lại được.
- **F5** Heartbeat giữa PC và xe. Mất kết nối quá `link_timeout_ms` thì xe dừng an toàn.

### 2.2 Bản đồ và vẽ đường
- **F6** Bản đồ lưới N×M, kích thước ô thực tế (mm) cấu hình được.
- **F7** Người dùng vẽ đường theo đỉnh lưới bằng cách click hoặc kéo. v1 chỉ cho đoạn ngang/dọc (góc cua 90°). v2 thêm 45° **[GĐ]**.
- **F8** Đặt vị trí xuất phát và hướng ban đầu. Vị trí xe đuôi được tính tự động: lùi lại `gap` dọc theo đường, hoặc người dùng đặt tay.
- **F9** Kiểm tra đường trước khi gửi: không tự cắt, đoạn thẳng đủ dài giữa 2 góc cua, `gap` ≥ khoảng cách an toàn khi quay tại chỗ (xem [algorithms.md §5](algorithms.md#5-giao-thức-qua-góc-cua-stop-and-go)), đường nằm trong bản đồ.
- **F10** Lưu và mở lại bản đồ, đường đi, cấu hình dạng file JSON.
- **F11** Xem trước bằng mô phỏng 2 xe chạy trên đường trước khi chạy thật.

### 2.3 Cấu hình
- **F12** Cấu hình hình học và động học của xe, cảm biến, bộ điều khiển (danh sách đầy đủ ở [parameters.md](parameters.md)). Có thể dùng chung cho 2 xe hoặc riêng từng xe.
- **F13** Đặt khoảng cách giữa 2 xe (`gap`) khi đi thẳng và khi qua góc cua.
- **F14** Có các bài hiệu chuẩn: chạy thẳng 1 m, quay 360°, đo cảm biến ở khoảng cách đã biết. Kết quả gợi ý giá trị bánh xe, track width và offset cảm biến.

### 2.4 Chạy và giám sát
- **F15** Lệnh: `LOAD_PATH`, `ARM`, `START`, `PAUSE`, `RESUME`, `STOP`, `E-STOP`, và chạy tay (joystick) để test.
- **F16** Chỉ START khi cả 2 xe đã ACK cùng `path_id` và checksum.
- **F17** Telemetry 10–20 Hz: pose ước lượng, tiến độ `s` trên đường, vận tốc 2 bánh, giá trị 2 cảm biến, gap ước lượng, trạng thái máy trạng thái, pin.
- **F18** Hiển thị realtime 2 xe trên bản đồ, đồ thị gap/vận tốc. Ghi log ra file để phân tích.

### 2.5 Hành vi chuyển động
- **F19** Xe đầu chạy theo đường với profile vận tốc: chạy nhanh trên đoạn thẳng, giảm tốc khi gần góc cua, dừng, quay tại chỗ, đi tiếp.
- **F20** Xe đuôi bám xe đầu và giữ gap mong muốn trên đoạn thẳng, bằng cách kết hợp đường biết trước, cảm biến và trạng thái xe đầu.
- **F21** Qua góc cua theo kiểu **stop-and-go** từng bước: xe 1 quay tại chỗ trong lúc xe 2 chờ. Xe 1 quay xong tiến lên một đoạn, rồi xe 2 tiến đến điểm cua và quay.
- **F22** Mất xe đầu khỏi cảm biến ngoài lúc quay (vượt ngưỡng thời gian), hoặc gap < `gap_min_safe`, thì xe đuôi dừng và báo lỗi.

## 3. Yêu cầu phi chức năng

| Mã | Yêu cầu | Mục tiêu v1 **[GĐ]** |
|---|---|---|
| N1 | Sai số gap trên đoạn thẳng | ±2 cm (ổn định) |
| N2 | Sai số ngang khi bám đường | ≤ 2 cm sau 2 m |
| N3 | Sai số góc sau mỗi lần quay 90° | ≤ 2° (cần IMU) |
| N4 | Tần số vòng điều khiển bánh | 100 Hz |
| N5 | Tần số đọc cảm biến khoảng cách | ≥ 30 Hz |
| N6 | Độ trễ leader→follower | < 30 ms (ESP-NOW), < 100 ms (qua PC) |
| N7 | An toàn | Dừng khi mất link, E-STOP từ web, watchdog firmware |
| N8 | Testability | Lõi thuật toán chạy được trên PC (unit test và sim) |

## 4. Tiêu chí nghiệm thu v1

1. Web dò được 2 xe, gán vai trò, gửi đường thành công.
2. Đường hình chữ "L" và hình chữ nhật 2×1 m: cả 2 xe chạy hết, không va chạm, xe đuôi cua đúng điểm cua (±3 cm).
3. Đi thẳng 3 m: gap giữ trong ±2 cm sau giai đoạn quá độ.
4. Rút WiFi hoặc bấm E-STOP thì cả 2 xe dừng trong < 300 ms.

# Yêu cầu

Nguồn: [brief gốc](../raw/2026-11-20-brief.md), [trả lời đợt 1](../raw/2026-11-20-answers-1.md), [đợt 2](../raw/2026-11-20-answers-2.md), [đợt 3](../raw/2026-11-20-answers-3.md).
Những điểm còn mở được đánh dấu **[GĐ]** và liệt kê ở [open-questions.md](open-questions.md).

## 0. Ràng buộc đã chốt
| # | Ràng buộc |
|---|---|
| C1 | Web Control chạy trên **PC**. ESP32 chỉ có API và không có UI chính |
| C2 | Mọi giao tiếp giữa 2 xe **đi qua PC**, không có link trực tiếp (không ESP-NOW) |
| C3 | Cảm biến mỗi xe: **2× ToF200C** (laser ToF, nhân VL53L0X) ở đầu xe và **1× IMU MPU6050** |
| C4 | Đường chỉ gồm đoạn ngang/dọc theo lưới, **góc cua đúng 90°**. Không có góc khác và không quay đầu 180° |
| C5 | Phần cứng có sẵn: **ESP32 DevKit thường (WROOM)**, driver **L298N**, 2 động cơ **GA25-370 12 V có encoder Hall** mỗi xe. GPIO sẽ được cung cấp sau ([hardware-intake](../work/hardware/hardware-intake.md)) |
| C6 | Thuật toán phải chạy được **hoàn toàn trên PC (mô phỏng 2 xe)** trước khi chạy trên xe. **Cùng một view** dùng cho sim, xe thật realtime và replay |
| C7 | **GPIO cố định trong code** (`pins.h`). Kích thước xe, khoảng cách, hệ số điều khiển/lọc, hiệu chuẩn thì **nhập trên web, gửi xuống ESP32, firmware áp dụng lúc chạy, lưu flash**. Không cần nạp lại firmware khi chỉnh |

## 1. Phạm vi
| Thành phần | Mô tả |
|---|---|
| Robot ×2 | ESP32, thân hình hộp, 3 bánh: 2 bánh sau dẫn động có encoder, 1 bánh nhỏ phía trước để cân bằng. 2 cảm biến khoảng cách ở mặt trước |
| Web Control | Server Python và frontend browser trên PC. Dò xe, cấu hình, vẽ đường, **điều phối góc cua**, chuyển tiếp dữ liệu giữa 2 xe, giám sát |
| Mạng | Cả 2 ESP32 và PC nằm trong cùng mạng WiFi 2.4 GHz (LAN) |

Ngoài phạm vi v1: tránh vật cản lạ (chỉ dừng khẩn cấp), định vị tuyệt đối, nhiều hơn 2 xe, đường cong.

## 2. Yêu cầu chức năng

### 2.1 Dò tìm và kết nối
- **F1** ESP32 tự vào WiFi biên dịch trong `secrets.h`, ưu tiên trước WiFi dự phòng trong NVS. Không phát AP; nếu không kết nối được thì chạy offline và dùng console USB để khôi phục.
- **F2** Web Control dò IP các xe trong LAN bằng **quét HTTP** `GET /api/info` (subnet, IP, IP:cổng, dải cổng). Mọi giao tiếp với robot thật là **HTTP** ([protocol.md](protocol.md), D25).
- **F3** Danh sách xe: tên, MAC, IP, fw, pin, RSSI, độ trễ ping. Có nút **Identify** (nháy LED và cho bánh lắc nhẹ) để biết xe nào là xe nào.
- **F4** Người dùng gán **LEADER** và **FOLLOWER**. Vai trò lưu ở cả PC và NVS của xe.
- **F5** Heartbeat PC↔xe. Xe mất PC quá `link_timeout_ms` thì phanh và dừng. PC mất một xe thì ra lệnh dừng xe còn lại.

### 2.2 Bản đồ và vẽ đường
- **F6** Lưới N×M, kích thước ô thực (mm) cấu hình được.
- **F7** Vẽ bằng cách click các đỉnh lưới. Mỗi đoạn tự ép về ngang hoặc dọc. Tại mỗi đỉnh nối chỉ cho **rẽ trái 90°, rẽ phải 90° hoặc đi thẳng** (các đoạn thẳng hàng tự gộp lại). **Không cho vẽ đường khép kín**: trình vẽ chặn điểm chạm lại đường cũ, validator cũng kiểm tra. Có undo/redo, xoá điểm, kéo điểm.
- **F8** Đặt điểm xuất phát và hướng ban đầu của leader. Follower tự được đặt lùi `gap` dọc trên phần kéo dài của đoạn đầu tiên (hiển thị trên lưới).
- **F9** Validator chạy trước khi gửi đường. Chi tiết quy tắc ở [web-ui.md §3](web-ui.md#3-validator):
  không tự cắt, không có góc 180°, mọi đoạn đều ≥ `seg_min`, có đủ khoảng trống quanh góc cua để quay tại chỗ, follower bắt đầu trong bản đồ.
- **F10** Lưu và mở lại project JSON (bản đồ, đường, cấu hình, vai trò).
- **F11** Mô phỏng 2 xe chạy trên đường trước khi chạy thật.

### 2.3 Cấu hình
- **F12** Cấu hình hình học, động cơ, cảm biến, bộ điều khiển ([parameters.md](parameters.md)). Mỗi xe một profile, copy được từ xe kia.
- **F13** Gap trên đoạn thẳng, gap lúc chờ ở góc cua, khoảng thoát sau khi cua.
- **F14** Các bài hiệu chuẩn: kiểm tra chiều motor và encoder, quét vùng chết PWM, đo bias và scale gyro, chạy thẳng 1 m, quay 4×90° và 360°, đo ToF ở 3 khoảng cách đã biết.

### 2.4 Chạy và giám sát
- **F15** Lệnh: `LOAD_PATH`, `ARM`, `START`, `PAUSE`, `RESUME`, `STOP`, `E-STOP`, cùng chế độ chạy tay (joystick) cho từng xe.
- **F16** Chỉ START khi cả 2 xe đã ACK cùng `path_id` và checksum.
- **F17** Telemetry 20–25 Hz: pose odometry, `s`, vận tốc 2 bánh, PWM, 2 số đo cảm biến, gap ước lượng, góc lệch tương đối, trạng thái, pin.
- **F18** Hiển thị realtime trên lưới, có bảng **bước góc cua** (bước nào đang chạy), đồ thị gap và vận tốc, ghi log JSONL.

### 2.5 Hành vi chuyển động
- **F19** Leader đi theo đường. Trên đoạn thẳng, leader giữ thẳng bằng đồng bộ encoder 2 bánh, có profile tăng/giảm tốc, chậm dần và dừng đúng điểm cua.
- **F20** Follower bám leader trên đoạn thẳng **tự chủ trên xe**. Gap và góc lệch lấy từ 2 cảm biến, đường biết trước và encoder. Không phụ thuộc vào độ trễ WiFi.
- **F21** Góc cua chạy theo quy trình **stop-and-go do PC điều phối** ([algorithms.md §5](algorithms.md#5-quy-trình-qua-góc-cua-pc-điều-phối)): leader dừng và quay, tiến ra. Follower đo lại khoảng cách rồi tiến tới điểm cua, quay, bắt lại leader và căn thẳng hàng. Sau đó cả 2 đi tiếp.
- **F23** Xử lý motor mạnh yếu và ma sát khác nhau: hiệu chuẩn riêng từng bánh, vòng tốc độ riêng từng bánh, vòng hướng theo gyro, xử lý bão hoà, thích nghi ([motor-control.md](motor-control.md)).
- **F22** An toàn tại xe: cảm biến thấy vật quá gần (`gap_min_safe` với follower, `obstacle_stop` với leader) thì phanh ngay, không chờ PC. Follower mất leader trên đoạn thẳng quá `lost_timeout_ms` thì dừng và báo PC.

### 2.6 Mô phỏng và view ([simulation.md](simulation.md))
- **F24** Sim chạy 2 robot ảo trên PC, dùng **chính code firmware** (thay HAL) và nói **đúng giao thức** của xe thật.
- **F25** View realtime chung cho các nguồn SIM, REAL, REPLAY. Ở sim có thêm lớp truth để so ước lượng với thật.
- **F26** Có điều khiển sim (tốc độ, pause, step, seed), thanh trượt tham số vật lý, và nút gây lỗi.
- **F27** Chế độ batch chạy kịch bản YAML, xuất metric, có tiêu chí pass/fail, dùng làm hồi quy.

### 2.7 Cấu hình động ([config-management.md](config-management.md))
- **F28** Form cấu hình trên web sinh tự động từ một schema chung. Có Apply (RAM), Save (flash), Revert, Diff, Export/Import, Factory reset.
- **F29** Firmware nhận tham số lúc chạy: validate, tính lại các giá trị suy ra, áp dụng ở ranh giới chu kỳ điều khiển (không giật, không lẫn cũ với mới). Hệ số chỉnh được bằng slider khi đang chạy (RAM). Tham số hình học chỉ đổi khi xe đứng yên.
- **F30** Telemetry mang `cfg_rev`, để view và log biết tham số nào đang có hiệu lực ở từng thời điểm.

## 3. Yêu cầu phi chức năng
| Mã | Yêu cầu | Mục tiêu v1 |
|---|---|---|
| N1 | Sai số gap trên đoạn thẳng (ổn định) | ±2 cm |
| N2 | Follower giữ song song với leader | góc lệch tương đối ≤ 2° |
| N3 | Sai số quay 90° (encoder + gyro MPU6050) | ≤ 1.5°. Follower được căn lại theo ToF |
| N4 | Vòng điều khiển bánh | 100 Hz |
| N5 | Đọc ToF200C | ≥ 30 Hz mỗi cảm biến (timing budget 33 ms) |
| N6 | Độ trễ relay xe→PC→xe | trung bình < 50 ms, p99 < 200 ms. Không dùng cho vòng điều khiển nhanh |
| N7 | An toàn | E-STOP từ web và dừng khi mất link < 300 ms. Watchdog firmware |
| N8 | Testability | Lõi thuật toán (C++) chạy trên PC: unit test và sim |
| N9 | Bền vững với sai lệch | Đạt N1–N3 trong sim khi lệch motor ±20 %, ma sát ±30 %, pin 12.6 V xuống 10.8 V |
| N10 | Sim | Deterministic theo seed. Batch nhanh hơn realtime ≥ 20× |

**Lưu ý về sai số hướng tuyệt đối:** gyro MPU6050 (sau khi trừ bias) trôi khoảng vài độ mỗi phút, nên hướng leader vẫn trôi chậm trên đường dài.
Hình dạng đường thực tế có thể lệch nhẹ so với bản vẽ. Follower không bị cộng dồn lỗi này vì nó căn theo leader bằng ToF.

## 4. Tiêu chí nghiệm thu v1
1. Web dò được 2 xe, Identify đúng xe, gán vai trò, gửi đường thành công.
2. Đường chữ "L" và hình chữ nhật 2×1 m: cả 2 xe chạy hết, không va chạm, follower quay tại điểm leader đã quay (±3 cm).
3. Đi thẳng 3 m: gap giữ ±2 cm sau giai đoạn quá độ.
4. E-STOP trên web hoặc tắt WiFi router: cả 2 xe dừng < 300 ms.

# Quản lý cấu hình: nhập trên web, đẩy xuống ESP32, lưu flash

Nguồn: [đợt 3](../raw/2026-11-20-answers-3.md). Danh sách tham số: [parameters.md](parameters.md).

## 1. Mục tiêu
- Mọi kích thước, offset cảm biến, hệ số điều khiển, kể cả **GPIO**, đều **nhập trên web**, gửi xuống ESP32 và **lưu flash**. Không cần nạp lại firmware khi chỉnh.
- Tinh chỉnh nhanh lúc dev: thử trên RAM trước, thấy ổn mới lưu flash.
- Sim dùng **đúng file cấu hình đó**.

## 2. Một schema duy nhất
`work/shared/config.schema.json` định nghĩa mỗi khoá với các thuộc tính: `type`, `unit`, `min`, `max`, `default`, `group`, `apply` (`live` | `idle` | `reboot`), `desc`.
Từ schema này sinh code (codegen) cho 3 phía:
| Đích | Sinh ra |
|---|---|
| Firmware/sim (C++) | `struct Config`, giá trị mặc định, hàm validate (min/max, ràng buộc chéo), (de)serialize ArduinoJson |
| Server (Python) | model pydantic, validate, diff |
| Web (TS) | kiểu dữ liệu và **form tự sinh** theo nhóm, có đơn vị, min/max, tooltip |

Thêm tham số mới chỉ cần sửa schema rồi chạy codegen. Không phải sửa tay ở 3 chỗ.

## 3. Lưu trữ trên ESP32
| Dữ liệu | Nơi lưu | Lý do |
|---|---|---|
| Danh tính, vai trò, WiFi | NVS | nhỏ, ít đổi |
| Cấu hình chính | **LittleFS** `/cfg/config.json` (~4–8 KB) cùng `/cfg/config.bak` | đọc được, dễ debug, export nguyên file |
| Hiệu chỉnh học được (L4) | LittleFS `/cfg/learned.json` | tách riêng để reset độc lập |

- **Ghi an toàn:** ghi `config.tmp`, đọc lại kiểm tra CRC, đổi `config.json` thành `.bak`, rồi đổi `tmp` thành `config.json`.
- **Lúc boot:** đọc `config.json` và validate. Lỗi thì dùng `.bak`. Lỗi tiếp thì dùng mặc định, bật cờ `CFG_DEFAULT` và báo lên web.
- **Safe mode cho GPIO:** đếm số lần boot liên tiếp bị crash (lưu trong RTC memory). Quá 3 lần thì bỏ qua nhóm `hw.*` và dùng chân mặc định.
- Mỗi lần lưu tăng `cfg_rev` và có `crc32`. Web dùng 2 giá trị này để biết PC và xe có khớp nhau không.

## 4. Áp dụng: khi nào có hiệu lực
| `apply` | Ví dụ | Hành vi |
|---|---|---|
| `live` | PID, hệ số follow, KF, `v_cruise`, gap, offset ToF | áp dụng ngay, kể cả khi đang chạy (nếu bật `allow_tune_while_running`) |
| `idle` | kích thước xe, bánh, track, `x_s`, `baseline` | chỉ áp dụng khi xe ở trạng thái IDLE. Đang chạy thì trả `nack busy` |
| `reboot` | `hw.pins.*`, I²C, tần số PWM, địa chỉ ToF | lưu flash và báo "cần khởi động lại". Có nút Reboot trên web |

Firmware **luôn validate lại** (không tin web): min/max, chân cấm (6–11, chân input-only không làm output được, chân trùng nhau), ràng buộc chéo (`g_hold ≥ g_hold_min`...).

## 5. Luồng trên web
```
Sửa form ─► [Apply]   PUT /api/config?persist=0   → RAM, có hiệu lực (theo apply)
         ─► [Save]    POST /api/config/save        → flash; xe trả {cfg_rev, crc}
         ─► [Revert]  POST /api/config/reload      → đọc lại từ flash
         ─► [Pull] [Diff] [Export JSON] [Import JSON] [Factory reset]
Chỉnh nhanh (slider) ─► WS {"t":"cfg_set","key":"pid.wheel.kp","value":1.2}  (RAM, tối đa 10 lần/s)
```
- Ô nào đã sửa mà chưa Save thì có viền vàng. Ô nào khác giữa PC và xe thì có viền tím.
- Project trên PC giữ **bản chuẩn** cho từng xe (theo MAC). Khi xe kết nối, so `cfg_rev`/`crc`. Nếu khác, web hỏi chiều đồng bộ: PC → xe hay xe → PC.
- Kết quả từ wizard hiệu chuẩn ([motor-control §3](motor-control.md#3-hiệu-chuẩn-trên-web-wizard)) đi qua đúng luồng này (Apply, kiểm tra, Save).
- Robot ảo trong sim có cùng API, nhưng "flash" của nó là file `work/sim/state/<id>/config.json`.

## 6. API (bổ sung [protocol.md](protocol.md))
| Method | Path | Mô tả |
|---|---|---|
| GET | `/api/config/schema` | phiên bản schema mà firmware hỗ trợ |
| GET | `/api/config` | cấu hình đang dùng (RAM) kèm `cfg_rev`, `crc`, `dirty` |
| PUT | `/api/config?persist=0\|1` | cập nhật một phần (JSON merge patch). Trả lỗi theo từng khoá |
| POST | `/api/config/save` / `reload` / `factory_reset` | |
| POST | `/api/reboot` | |
| GET/PUT/DELETE | `/api/config/learned` | hiệu chỉnh học được |

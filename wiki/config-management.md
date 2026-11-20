# Quản lý cấu hình: tham số động từ web xuống ESP32

Nguồn: [đợt 3](../raw/2026-11-20-answers-3.md), [đợt 4](../raw/2026-11-20-answers-4.md). Danh sách tham số: [parameters.md](parameters.md).

## 1. Phạm vi
| Loại | Ở đâu | Đổi bằng cách nào |
|---|---|---|
| **GPIO, địa chỉ I²C, chọn bus** | **Cố định trong code**: `work/firmware/src/board/pins.h` | sửa code rồi nạp lại firmware |
| Kích thước xe, bánh, vị trí cảm biến, khoảng cách (gap...), tốc độ, hệ số điều khiển, bộ lọc, hiệu chuẩn | **Tham số động**: nhập trên web, gửi xuống xe, lưu flash | Web Control, không cần nạp lại firmware |

**Nguyên tắc firmware:** trong `lib/core` và `lib/app` **không có hằng số vật lý hay hệ số nào viết cứng**. Mọi giá trị đều đọc từ `Params` lúc chạy. Hằng số chỉ còn trong `pins.h` và giá trị mặc định do codegen sinh ra.

## 2. Một schema duy nhất
`work/shared/config.schema.json` định nghĩa mỗi khoá với các thuộc tính: `type`, `unit`, `min`, `max`, `default`, `group`, `apply` (`live` | `idle`), `desc`.
Từ schema này sinh code (codegen) cho 3 phía:
| Đích | Sinh ra |
|---|---|
| Firmware/sim (C++) | `struct Config`, giá trị mặc định, validate (min/max, ràng buộc chéo), (de)serialize ArduinoJson, bảng `key → offset/type/group` để cập nhật từng khoá |
| Server (Python) | model pydantic, validate, diff |
| Web (TS) | kiểu dữ liệu và **form tự sinh** theo nhóm, có đơn vị, min/max, tooltip |

## 3. Firmware xử lý tham số động

### 3.1 Luồng xử lý
```
 HTTP PUT /api/config  ─┐
 WS cfg_set (slider)    ├─► ParamService (task net, core 0)
 Boot: LittleFS         ─┘      1. parse JSON patch → bản copy Config mới (staging)
                                2. validate từng khoá + ràng buộc chéo → lỗi trả theo khoá
                                3. kiểm tra apply: có khoá 'idle' mà xe đang chạy → nack busy
                                4. computeDerived(Config) → Derived
                                5. đẩy {Config, Derived, ChangeSet} vào slot "pending"
                                         │
 control task (100 Hz, core 1), đầu mỗi chu kỳ:
   if pending: swap active ↔ pending (con trỏ, không copy) → gọi onParams() các module
   chạy thuật toán với *active (bất biến trong suốt chu kỳ)
```
- **Không bao giờ bị "nửa cũ nửa mới":** mỗi chu kỳ điều khiển chỉ thấy một snapshot `Config` + `Derived` bất biến. Việc đổi snapshot xảy ra ở ranh giới chu kỳ (double buffer + cờ atomic, không cần khoá lâu).
- Task `range` và `imu` lấy snapshot theo cùng cơ chế (đọc con trỏ active khi bắt đầu mỗi lần xử lý).
- `cfg_rev` tăng sau mỗi lần áp dụng thành công và được gửi trong **mọi gói telemetry**. View và log biết chính xác tham số nào đang có hiệu lực ở từng thời điểm, để replay hoặc so sánh.

### 3.2 Tham số suy ra (`Derived`), tính lại mỗi lần áp dụng
| Derived | Tính từ |
|---|---|
| `mm_per_tick_l/r` | `wheel.diameter`, `diam_eps`, `enc.ppr_motor`, `enc.gear_ratio` |
| Ma trận động học `(v,ω) ↔ (v_l,v_r)` | `wheel.track_eff` |
| `R_sw`, `g_hold_min`, `exit_dist_min`, `gap.min_safe` (auto) | `body.*`, `corner.margin`, `range.x_mm` |
| Hình học ToF: `x_s`, `b`, `x_rear` của leader | `range.*`, `body.axle_to_rear` |
| Bảng tra FF nghịch, `v_max`, `v_min_reliable` | `motor.lut_*`, `power.*` |
| Hệ số PI rời rạc theo `dt` | `pid.*`, `motor.model_*` |
| Ma trận `Q`, `R` của KF, ngưỡng gating | `kf.*`, `range.sigma_mm` |
| Profile vận tốc của đường hiện tại | `motion.*` cùng path đã nạp |

### 3.3 Module phản ứng khi tham số đổi
Mỗi module có hàm `onParams(const Config&, const Derived&, ChangeSet)`. `ChangeSet` là bitmask theo nhóm, để module chỉ làm việc khi nhóm của nó thật sự đổi.
| Module | Khi nhóm của nó đổi |
|---|---|
| PI bánh, PI hướng, follow | **Chuyển không giật (bumpless):** quy đổi lại tích phân sao cho đầu ra liên tục với hệ số mới |
| Odometry | Giữ nguyên pose. Từ tick tiếp theo dùng `mm_per_tick` mới |
| KF gap | Đổi `Q`/`R` mà giữ trạng thái. Nếu đổi hình học ToF thì reset KF bằng số đo kế tiếp |
| Profile vận tốc | Tính lại từ vị trí `s` hiện tại, giới hạn gia tốc để không giật |
| Motor FF | Dùng LUT mới ngay. Offset thích nghi được giữ nếu cùng nhóm |
| PWM | `ledcChangeFrequency` khi idle |

### 3.4 Live và idle
| `apply` | Nhóm | Hành vi |
|---|---|---|
| `live` | `pid.*`, `follow.*`, `kf.*`, `motion.v_cruise/a_max/v_corner`, `gap.target`, `range.offset/scale/sigma`, `motor.adapt.*` | áp dụng ngay cả khi đang chạy (nếu bật `safety.allow_tune_while_running`) |
| `idle` | `body.*`, `wheel.*`, `enc.*`, `range.x_mm/baseline`, `corner.*`, `gap.hold`, `motor.lut_*`, `motor.pwm_hz` | chỉ khi xe IDLE. Đang chạy thì trả `nack busy`, hoặc xếp hàng chờ áp dụng khi về IDLE (tuỳ chọn `queue=1`) |

Khi đổi nhóm hình học mà đường đi đã nạp, PC tự **nạp lại path** (profile và validator phụ thuộc kích thước).

### 3.5 Lúc boot và an toàn
- Đọc LittleFS, validate rồi tính `Derived`. Lỗi thì dùng `.bak`. Lỗi tiếp thì dùng mặc định và bật cờ `CFG_DEFAULT`.
- Khi đang có cờ `CFG_DEFAULT` hoặc `UNCALIBRATED` (chưa có LUT motor), lệnh `ARM` bị từ chối. Chỉ cho phép jog chậm và các bài hiệu chuẩn.
- Schema có `schema_ver`. Khoá lạ thì bỏ qua và cảnh báo. Khoá thiếu thì lấy mặc định. Có hàm migrate khi tăng phiên bản.

## 4. Lưu trữ trên ESP32
| Dữ liệu | Nơi lưu |
|---|---|
| Danh tính, vai trò, WiFi | NVS |
| Tham số động | LittleFS `/cfg/config.json` (~4–8 KB) cùng `/cfg/config.bak` |
| Hiệu chỉnh học được (L4) | LittleFS `/cfg/learned.json` (reset độc lập) |

Ghi atomic: ghi `config.tmp`, kiểm tra CRC, đổi bản cũ thành `.bak`, rồi đổi `tmp` thành `config.json`. Chỉ ghi flash khi bấm **Save**. Apply và slider chỉ ghi RAM, không làm mòn flash.

## 5. Luồng trên web
```
Sửa form ─► [Apply]   PUT /api/config?persist=0   → RAM, có hiệu lực (live/idle)
         ─► [Save]    POST /api/config/save        → flash; xe trả {cfg_rev, crc}
         ─► [Revert]  POST /api/config/reload      → đọc lại từ flash
         ─► [Pull] [Diff] [Export JSON] [Import JSON] [Factory reset]
Slider ─► WS {"t":"cfg_set","key":"pid.wheel.kp","value":1.2}  (RAM, tối đa 10 lần/s)
```
- Ô chưa lưu flash có viền vàng. Ô khác giữa PC và xe có viền tím. Khoá `idle` có biểu tượng khoá khi xe đang chạy.
- Project trên PC giữ **bản chuẩn** cho từng xe (theo MAC). Khi xe kết nối, so `cfg_rev`/`crc`. Nếu khác, web hỏi chiều đồng bộ: PC → xe hay xe → PC.
- Wizard hiệu chuẩn ([motor-control §3](motor-control.md#3-hiệu-chuẩn-trên-web-wizard)) đi qua đúng luồng này.
- Robot ảo có cùng API, "flash" của nó là `work/sim/state/<id>/config.json`. Nhờ vậy cơ chế tham số động được **test trong sim trước**, ví dụ kịch bản đổi `pid.*` và `gap.target` giữa lúc chạy.

## 6. API (bổ sung [protocol.md](protocol.md))
| Method | Path | Mô tả |
|---|---|---|
| GET | `/api/config/schema` | phiên bản schema mà firmware hỗ trợ |
| GET | `/api/config` | cấu hình đang dùng kèm `cfg_rev`, `crc`, `dirty`, và `derived` (chỉ đọc, để hiển thị) |
| PUT | `/api/config?persist=0\|1&queue=0\|1` | cập nhật một phần (JSON merge patch). Trả `{ok, cfg_rev, errors:{key:msg}, deferred:[keys]}` |
| POST | `/api/config/save` / `reload` / `factory_reset` | |
| GET/PUT/DELETE | `/api/config/learned` | hiệu chỉnh học được |

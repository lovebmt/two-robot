# Kiến trúc

## 1. Tổng quan

```
            ┌──────────────── PC (LAN) ────────────────┐
            │  Browser UI  ⇄  Control Server (FastAPI)  │
            │  grid editor     discovery (mDNS/UDP/scan)│
            │  config/sim      path planner + validator │
            │  telemetry       WS hub, logger           │
            └───────────▲───────────────────▲───────────┘
              WiFi WS/HTTP                 WiFi WS/HTTP
            ┌───────────┴───────┐   ┌───────┴───────────┐
            │  ESP32 LEADER     │◄─►│  ESP32 FOLLOWER   │
            │                   │ESP│                   │
            │  path follower    │NOW│  path follower    │
            │  odometry+IMU     │   │  + gap estimator  │
            └───────────────────┘   └───────────────────┘
```

- **PC** lo phần "nặng" và không cần realtime: giao diện, lập đường, kiểm tra hợp lệ, mô phỏng, log.
- **ESP32** lo phần realtime: điều khiển bánh, ước lượng, bám xe. Chạy tự lập sau khi đã nhận đường. Mất PC thì dừng an toàn, không phụ thuộc PC trong từng vòng lặp.
- **ESP-NOW** là link trực tiếp leader→follower (20–50 Hz, độ trễ vài ms) mang tiến độ `s_L`, vận tốc và sự kiện góc cua. Fallback: relay qua WebSocket của PC.
- Brief có nhắc "web server của 1 con". Thiết kế ở đây chọn **web chạy trên PC**, mỗi xe chỉ có API nhỏ, vì ESP32 không đủ tài nguyên cho UI lưới, sim và log. Xem [open-questions Q1](open-questions.md).

## 2. Phần cứng (đề xuất, mỗi xe) **[GĐ]**

| Khối | Đề xuất | Ghi chú |
|---|---|---|
| MCU | ESP32-WROOM-32 / ESP32-S3 DevKit | S3 có nhiều chân hơn |
| Động cơ | 2× N20 hoặc JGA25-370 có encoder Hall 2 kênh | Biết CPR và tỉ số truyền |
| Driver | TB6612FNG (hoặc DRV8833) | PWM 20 kHz bằng LEDC |
| Encoder | Bộ đếm **PCNT** phần cứng của ESP32, chế độ x4 | Không dùng ngắt GPIO |
| Cảm biến khoảng cách ×2 | **VL53L1X** hoặc VL53L0X (ToF, I²C) | Cần chân XSHUT để đổi địa chỉ I²C. Siêu âm HC-SR04 chậm và chùm rộng, không khuyến nghị |
| IMU | MPU6050 / ICM-42688 / BNO085 (khuyên dùng) | Gyro Z rất cần cho quay tại chỗ chính xác |
| Nguồn | 2S Li-ion 7.4 V + buck 5 V | Đo pin bằng ADC qua cầu chia |
| Khác | LED trạng thái, buzzer, nút E-STOP vật lý | |
| Tấm phản xạ đuôi | Tấm phẳng mờ, sáng màu, ở mặt sau xe, rộng ≥ khoảng cách 2 cảm biến + biên | Rất quan trọng để đo ToF ổn định |

Bố trí: bánh dẫn động ở phía sau, trục bánh là gốc toạ độ xe. Caster ở trước. Cảm biến ở mặt trước, đối xứng qua trục dọc, cách nhau `sensor_baseline`.
Xe quay tại chỗ quanh tâm trục bánh, nên **bán kính quét** là khoảng cách từ tâm trục tới góc xa nhất của thân xe. Thông số này quyết định gap an toàn khi cua.

## 3. Firmware (ESP32)

- PlatformIO, framework Arduino-ESP32 (v3.x), FreeRTOS tasks. Xem [ops/decisions D2](../ops/decisions.md).
- **`lib/core/` là C++17 thuần, không include Arduino.** Gồm kinematics, odometry, path, profile vận tốc, estimator, controller, state machine. Phần này build được cho `native` để unit test và chạy sim.
- `src/hal/` gồm motor, encoder, ToF, IMU, NVS. `src/net/` gồm WiFi, mDNS, HTTP, WS, ESP-NOW.

| Task | Tần số | Core | Việc |
|---|---|---|---|
| `control` | 100 Hz | 1 | đọc encoder và IMU, odometry, estimator, controller, PWM |
| `sensors` | 30–50 Hz | 1 | đọc 2 ToF (continuous mode), lọc, timestamp |
| `peer` | 20–50 Hz | 0 | ESP-NOW tx/rx trạng thái |
| `net` | sự kiện | 0 | HTTP/WS, telemetry 10–20 Hz |
| `safety` | 50 Hz | 1 | watchdog, timeout link, pin yếu, E-STOP |

## 4. Control Web (PC)

- **Server: Python 3.11+ FastAPI** cùng `zeroconf` (mDNS), UDP listener, async subnet scan, WebSocket hub tới browser và các xe, planner/validator, logger (JSONL/CSV).
  Phải có server vì browser không tự dò LAN được (không có mDNS/UDP).
- **Frontend: Vite + TypeScript + Canvas 2D**, không cần framework nặng. Các màn hình: *Devices*, *Config*, *Map & Path*, *Run/Monitor*, *Calibrate*.
- **Sim:** dùng chính `lib/core` C++, build bằng native (CLI xuất JSON trajectory) và sau này build WASM để chạy ngay trong browser. Xem [decisions D4](../ops/decisions.md).

## 5. Giao thức

### 5.1 Discovery
- Xe quảng bá mDNS `tworobot-<mac4>.local` với service `_tworobot._tcp` port 80, TXT: `id, role, fw`.
- Xe gửi UDP beacon broadcast port `47800` mỗi 1 s, nội dung JSON `{id, ip, role, fw, bat}`.
- Fallback: PC quét `GET http://<ip>/api/info` trên toàn subnet /24 (timeout 300 ms, chạy song song).

### 5.2 PC ⇄ Robot
- HTTP REST (cấu hình, nạp đường, payload lớn):
  `GET /api/info`, `GET/PUT /api/config`, `PUT /api/role`, `PUT /api/path`, `POST /api/calib/{test}`, `POST /api/identify`.
- WebSocket `/ws` (lệnh và telemetry, JSON, sau này có thể đổi sang CBOR):
  `{"t":"cmd","op":"start","path_id":..}` · `{"t":"hb"}` · `{"t":"tel", ...}` · `{"t":"evt","e":"corner_done","k":3}`.
- Mọi message có `seq` và `ts`. Lệnh quan trọng cần ACK.

### 5.3 Leader → Follower (ESP-NOW, ~50 Hz, struct nhị phân)
```
struct PeerState { uint8 ver; uint16 seq; uint32 t_ms; uint16 path_id;
                   float s;      // tiến độ trên đường (mm)
                   float v;      // vận tốc dài (mm/s)
                   float yaw;    // rad
                   uint8 seg;    // chỉ số đoạn/góc hiện tại
                   uint8 phase;  // DRIVE/APPROACH/STOPPED/TURNING/EXIT/...
                   uint8 flags; } // estop, fault...
```
Follower gửi ngược `{s, phase, corner_ack}` để bắt tay khi qua góc cua.

### 5.4 Định dạng đường đi (PC → xe)
PC chuyển nét vẽ trên lưới thành **danh sách đoạn theo toạ độ thực (mm)** và các thao tác:
```json
{ "path_id": 12, "units": "mm",
  "start": {"x":0,"y":0,"yaw_deg":0},
  "ops": [ {"op":"line","len":1200},
           {"op":"turn","deg":90},
           {"op":"line","len":600} ],
  "corners_s": [1200],
  "follower_start_s": -350,
  "checksum": "crc32..." }
```
Hai xe nhận **cùng một đường**. Xe đuôi bắt đầu ở `s = -gap` (phần đường kéo dài lùi về sau điểm xuất phát).

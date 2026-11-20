# Kiến trúc

## 1. Tổng quan

```
┌────────────────────────── PC (LAN) ──────────────────────────┐
│ Browser UI ⇄ Control Server (Python FastAPI)                 │
│  grid editor   ├ discovery (quét HTTP GET /api/info)         │
│  config/sim    ├ planner + validator                         │
│  run/monitor   ├ RELAY: leader state → follower (HTTP POST)  │
│                ├ CORNER COORDINATOR (máy trạng thái)         │
│                ├ time sync, heartbeat, logger                │
└───────▲─────────────────────────────────────▲────────────────┘
        │ HTTP: POST /api/cmd (lệnh)          │
        │ HTTP: GET /api/state 20 Hz (poll)   │
┌───────┴──────────┐                  ┌───────┴──────────┐
│ ESP32 LEADER     │   (không nói     │ ESP32 FOLLOWER   │
│ drive/turn prims │    chuyện trực   │ follow (local),  │
│ obstacle stop    │    tiếp)         │ gap KF, prims    │
└──────────────────┘                  └──────────────────┘
```

**Phân chia trách nhiệm:**
- **Trên xe (realtime, tự chủ):** điều khiển bánh 100 Hz, đi thẳng, quay 90°, dừng tại điểm, và *follower bám leader trên đoạn thẳng bằng cảm biến*. Không vòng nhanh nào phụ thuộc WiFi.
- **Trên PC (sự kiện, chậm):** điều phối các bước tại góc cua, chuyển tiếp trạng thái leader cho follower (làm feedforward, không bắt buộc), giám sát và an toàn tổng.

Lý do chọn cách này: qua góc cua là chuỗi bước rời rạc, chạy chậm, nên chịu được độ trễ WiFi. Bám trên đoạn thẳng cần nhanh, nên chạy trên xe bằng cảm biến (đo trực tiếp, không trễ).

## 2. Phần cứng (đã có: ESP32 DevKit, L298N, GA25-370 encoder, 2× ToF200C, MPU6050)

Thông số cụ thể và GPIO sẽ được điền vào [hardware-intake.md](../work/hardware/hardware-intake.md).

### 2.1 L298N, những điểm cần lưu ý
- Mỗi motor dùng 3 chân: `ENx` (PWM), `INx1` và `INx2` (chiều). **Tháo jumper ENA/ENB** để PWM được.
- Driver dùng BJT nên **sụt áp ~1.5–2.5 V**. Nguồn motor phải cao hơn điện áp định mức của motor tương ứng (ví dụ motor 6 V dùng 2S 7.4–8.4 V).
- PWM bắt đầu ở **~1–5 kHz** (L298N đóng cắt chậm, ở 20 kHz bị nóng và mất mô-men), 10 bit, dùng LEDC. Sẽ có tiếng kêu nhẹ, chấp nhận được.
- Ba chế độ: tiến/lùi (`IN1≠IN2`), **phanh nhanh** (`IN1=IN2=HIGH`, EN=HIGH, dùng khi dừng ở góc cua), thả trôi (EN=LOW).
- Nguồn ESP32: nên dùng **buck 5 V riêng**. Chỉ dùng chân 5 V của L298N khi Vs ≤ 12 V và jumper 5V-EN đang cắm. Nguồn này nhiễu nên phải có tụ. **GND chung** giữa L298N, ESP32 và encoder.
- Vùng chết PWM lớn và không tuyến tính nên **cần feedforward cùng bù vùng chết** ([algorithms §1.2](algorithms.md#12-điều-khiển-bánh-l298n)).

### 2.2 Motor GA25-370 và encoder
- GA25-370 có encoder Hall 2 kênh, thường **11 PPR trên trục motor**. Số tick mỗi vòng bánh = `11 × 4 × gear`. Ví dụ hộp số 1:34 cho ~1496 tick/vòng, rất đủ độ phân giải.
- **Bản 12 V** (đã chốt). Dùng pin **3S Li-ion (10.8–12.6 V)**. Sau khi sụt áp qua L298N, motor chỉ nhận ~9–10.5 V, nên tốc độ tối đa còn khoảng 75–85 % định mức. Feedforward tính theo `V_bat` đo được ([motor-control L0](motor-control.md#2-giải-pháp-phân-tầng-từ-trong-ra-ngoài)). Tỉ số hộp số và rpm vẫn cần xác nhận (Q15).
- Motor mạnh yếu khác nhau và ma sát khác nhau: xử lý theo [motor-control.md](motor-control.md).
- Đếm bằng **PCNT phần cứng** (x4 quadrature, có glitch filter). Không dùng ngắt GPIO.
- Encoder thường cấp 3.3–5 V. **Cấp 3.3 V** thì không cần dịch mức (cần đo xác nhận đầu ra). Nếu đầu ra là 5 V thì phải chia áp về 3.3 V.

### 2.3 Cảm biến khoảng cách: ToF200C ×2
- ToF200C là module laser ToF dùng nhân **VL53L0X** (I²C, địa chỉ mặc định `0x29`), tầm đo khoảng 30–2000 mm, FOV ~25°. Cần xác nhận lại sơ đồ chân trên module thật.
- Hai module trùng địa chỉ nên **cần 2 chân XSHUT**. Lúc boot: giữ cả 2 ở reset, bật module trái và đổi sang `0x30`, sau đó bật module phải (giữ `0x29`).
- Chế độ continuous, timing budget 33 ms (~30 Hz). Có thể giảm xuống 20 ms (~50 Hz) nếu chấp nhận nhiễu tăng. Dùng chân INT/GPIO1 (nếu có) để biết khi nào có dữ liệu mới, đỡ phải poll.
- Mỗi chùm rộng ~25°, nên ở gap 300 mm vệt chùm rộng khoảng 130 mm. **Tấm phản xạ phải rộng hơn vệt chùm**, nếu không chùm sẽ "thấy" cả sàn hoặc vật bên cạnh. Có thể đặt 2 cảm biến cách nhau rộng (`b` lớn) và **nghiêng nhẹ vào trong** để giảm chồng vệt. Sẽ thử nghiệm ở M4.
- Lớp trừu tượng `RangeSensor` vẫn được giữ để sau này đổi loại cảm biến được.

**Tấm phản xạ đuôi:** mặt sau leader cần một tấm phẳng, cứng, mờ, sáng màu, rộng bằng thân xe, đặt cùng chiều cao với mắt ToF. Đây là yếu tố quyết định chất lượng bám.

### 2.3b IMU MPU6050
- Dùng **gyro trục Z** (yaw rate) là chính. Accelerometer chỉ dùng để phát hiện xe đứng yên hoặc bị va chạm.
- Cấu hình: ±250 dps, DLPF ~42 Hz, đọc 200 Hz (FIFO hoặc poll). Bias được đo khi đứng yên (2 s lúc boot và cập nhật lại mỗi lần dừng ở góc cua).
- Gắn gần tâm trục bánh, trục Z thẳng đứng, có đệm mút chống rung động cơ.
- **Bus I²C:** MPU6050 (`0x68`) và 2 ToF (`0x29`/`0x30`) dùng chung bus 400 kHz được. Tuy vậy nên tách **IMU ra bus I²C thứ 2** (ESP32 có 2 controller) để đọc gyro 200 Hz không bị chặn bởi ToF.

### 2.4 Ràng buộc GPIO của ESP32
GPIO **cố định trong code** tại `work/firmware/src/board/pins.h` (D19). File này có `static_assert` để chặn lỗi chọn chân lúc biên dịch: chân flash 6–11, chân input-only bị dùng làm output, chân trùng nhau.
- Tránh chân **6–11** (flash). Hạn chế chân strapping **0, 2, 5, 12, 15** (12 HIGH lúc boot sẽ treo).
- **34–39 chỉ là input**, không có pull-up nội. Dùng được cho encoder hoặc ECHO nếu có pull-up/chia áp ngoài.
- ADC2 không dùng được khi bật WiFi. Đo pin bằng ADC1 (32–39).
- I²C mặc định 21/22. LED trạng thái và buzzer dùng chân output bất kỳ.
- Tổng mỗi xe: 6 (L298N) + 4 (encoder) + 2 (I²C ToF) + 2 (XSHUT) + 0–2 (INT ToF) + 2 (I²C IMU) + 1 (ADC pin) + 1–2 (LED/buzzer) ≈ **18–21 chân**. ESP32 DevKit 30 chân vẫn đủ nếu tránh strapping.

## 3. Firmware (ESP32)
> **Đã cài đặt (2026-11-20):** `work/firmware`, xem [README](../work/firmware/README.md). Thực tế: platform cố định espressif32@6.8.1 (Arduino 2.0.17), WebServer đồng bộ (HTTP, D25), lõi `lib/core` không phụ thuộc Arduino, test native. Bản Python `tworobot/fw` là bản tham chiếu dùng trong sim. Sơ đồ dưới đây là thiết kế ban đầu.

- PlatformIO, Arduino-ESP32, FreeRTOS. Tách lớp để **cùng thuật toán chạy trên ESP32 và trên PC** ([simulation §2.1](simulation.md#21-tách-hal-trong-firmware)):

```
work/firmware/
  lib/core/     C++17 thuần: kinematics, motor ctrl (FF LUT, PI, bão hoà), fusion, KF, follow
  lib/app/      primitive, máy trạng thái, safety, scheduler,
                params/ (Config codegen, validate, Derived, ParamService, double-buffer swap)
  lib/proto/    encode/decode message, dispatcher lệnh (không phụ thuộc transport)
  lib/hal_api/  interface IMotor/IEncoder/IImu/IRange/IClock/IBattery/IStorage/ITransport
  src/board/pins.h  GPIO, địa chỉ I²C, port cố định (compile-time)
  src/hal_esp32/  LEDC, PCNT, Wire (2 bus), VL53L0X, MPU6050, LittleFS, HTTP server (AsyncTCP)
  src/main.cpp    tạo HAL ESP32 và FreeRTOS task, gọi lib/app
work/sim/
  hal_sim/  world/  main.cpp      (CMake, dùng lại lib/*)
```
- **Tham số động:** kích thước, khoảng cách, hệ số đều đọc từ snapshot `Config + Derived` lúc chạy. Snapshot được đổi ở ranh giới chu kỳ điều khiển. Chi tiết ở [config-management §3](config-management.md#3-firmware-xử-lý-tham-số-động).

| Task | Tần số | Core | Việc |
|---|---|---|---|
| `imu` | 200 Hz | 1 | đọc gyro Z, trừ bias, tích phân yaw |
| `control` | 100 Hz | 1 | encoder, fusion odometry + gyro, primitive hiện tại (drive/turn/follow), PI bánh, PWM |
| `range` | 30 Hz | 1 | đọc 2 ToF, lọc median/outlier, timestamp |
| `http` | sự kiện | 0 | HTTP server: lệnh/relay vào hàng đợi lock-free, trả snapshot telemetry + hộp thư sự kiện |
| `tel` | 25 Hz | 1 | chuẩn bị snapshot telemetry (handler HTTP chỉ copy, không tính toán) |
| `safety` | 50 Hz | 1 | heartbeat timeout, E-STOP, pin yếu, watchdog |

**WiFi:** `WiFi.setSleep(false)` (bắt buộc, nếu không độ trễ sẽ lên 100–300 ms), kênh cố định nếu được, có thể đặt DHCP reservation trên router.

### 3.1 Primitive trên xe (PC chỉ ra lệnh ở mức này)
| Primitive | Tham số | Kết thúc |
|---|---|---|
| `DRIVE_TO` | `s_end`, `v_max` | dừng (phanh) tại `s_end`, gửi `done` |
| `TURN` | `deg` (±90 hoặc góc nhỏ để căn) | quay xong, gửi `done` |
| `FOLLOW` | `g_target`, `s_limit`, `v_max` | follower bám tới khi đạt `s_limit` hoặc giữ ở `g_hold` |
| `SNAPSHOT` | `n_samples` | gửi lên trung bình `ρ`, `Δψ`, độ lệch chuẩn |
| `REACQUIRE` | `ρ_expected`, `tol` | tìm và xác nhận leader trước mặt, có thể quét góc nhỏ |
| `ALIGN` | `tol_deg` | quay tại chỗ tới khi `Δψ≈0` (theo cảm biến) |
| `HOLD` / `BRAKE` | | đứng yên và giữ phanh |

## 4. Control Web (PC)
- **Server:** Python 3.11+, FastAPI, asyncio. Gồm các module: `discovery` (quét HTTP), `links` (SimLink / NetLink HTTP poll 20 Hz), `relay`, `coordinator`, `planner`, `validator`, `timesync`, `logger`, `sim_bridge`.
- **Frontend:** Vite + TypeScript + Canvas 2D. Màn hình chi tiết ở [web-ui.md](web-ui.md).
- **Sim:** process C++ chạy 2 robot ảo (chính code firmware, thay HAL) cộng mô hình vật lý. Nói cùng giao thức nên server và UI không phân biệt sim với thật. Có cùng một view cho SIM, REAL và REPLAY. Chi tiết ở [simulation.md](simulation.md).

## 5. Mạng
- Router 2.4 GHz, **tắt AP/client isolation** (nếu bật, PC không thấy xe và broadcast bị chặn). Nên dùng router riêng cho sân test.
- Chi tiết message ở [protocol.md](protocol.md).

# Firmware ESP32: two-robot

Port C++ từ firmware ảo Python (`work/control-web/server/tworobot/fw`). Giao tiếp với Web Control **chỉ bằng HTTP qua LAN** (`wiki/protocol.md`).
Phần cứng: ESP32 DevKit, L298N, 2× GA25-370 có encoder Hall, 2× ToF200C (VL53L0X), MPU6050.

## Cấu trúc
```
lib/core/src/        C++17 thuần, KHÔNG phụ thuộc Arduino → test được trên máy tính
  config_gen.*       SINH TỰ ĐỘNG từ work/shared/config.schema.json (python3 work/shared/codegen_cpp.py)
  params.*           validate, ràng buộc chéo, giá trị suy ra, CRC
  paramsvc.*         tham số động: patch → validate → pending → swap đầu chu kỳ; lưu flash
  control.*          PI bánh + FF + bão hoà + học FF; calib gyro; ZUPT + bù nhiệt; odometry
  gapkf.h rangeproc.* bộ lọc gap; 2 ToF → ρ, Δψ
  primitives.*       DRIVE_TO TURN HOLD FOLLOW SNAPSHOT ALIGN REACQUIRE TOF_CAL CAL_DRIVE CAL_TURN
  app.*              máy trạng thái, an toàn (mất heartbeat, lỗi phần cứng), HTTP API, telemetry, hộp thư sự kiện
  hal.h              interface phần cứng
src/
  board/pins.h       GPIO CỐ ĐỊNH (⚠ kiểm tra theo dây thật)
  hal_esp32.*        L298N (LEDC), encoder (PCNT x4), MPU6050 (I2C 21/22), 2× VL53L0X (I2C 25/26 + XSHUT), LittleFS, NVS
  net.*              WiFi STA / AP cài WiFi, WebServer cổng 80
  main.cpp           task: control 100 Hz (core 1), range ~30 Hz (core 1), http (core 0)
test/test_core/      test native: params, KF, ToF, App; ĐỐI CHIẾU với Python (test/vectors/*.json)
tools/gen_vectors.py sinh dữ liệu đối chiếu từ firmware ảo Python
```

## Build, nạp, test
```bash
cd work/firmware
pio run -e esp32dev                       # build (platform cố định espressif32@6.8.1 = Arduino 2.0.17)
pio run -e esp32dev -t upload             # nạp qua USB
pio device monitor                        # log 115200: IP, trạng thái, thời gian tick
pio test -e native                        # 12 test lõi trên máy tính
../control-web/.venv/bin/python tools/gen_vectors.py   # sinh lại dữ liệu đối chiếu khi đổi thuật toán Python
```
Đổi schema tham số → chạy `python3 work/shared/codegen_cpp.py` rồi build lại. Server và web đọc chung schema đó.

## Đi dây (theo `src/board/pins.h`, bảng đã chốt với dây thật 2026-11-20)
| Chức năng | GPIO | Ghi chú |
|---|---|---|
| I2C MPU6050 SDA / SCL | **21 / 22** | bus 0 (Wire), địa chỉ 0x68 |
| I2C 2 ToF SDA / SCL | **25 / 26** | bus 1 (Wire1), riêng cho ToF; 2 ToF nối chung bus này (0x29 / 0x30) |
| ToF phải XSHUT | 32 | giữ địa chỉ 0x29 |
| ToF trái XSHUT | 33 | firmware đổi sang 0x30 lúc boot |
| Encoder phải A / B | 14 / 27 | |
| Encoder trái A / B | 13 / **35** | **không dùng GPIO12** (xem cảnh báo dưới). 35 không có pull-up nội |
| L298N ENA / IN1 / IN2 (motor trái) | 23 / 19 / 18 | **tháo jumper ENA** |
| L298N ENB / IN3 / IN4 (motor phải) | 17 / 4 / 16 | **tháo jumper ENB** |
| Đo pin | — | **không dùng** (xe không có cầu chia). Muốn đo thì dùng chân ADC1 32–39 và đặt `power.vbat_divider` |
| LED | 2 | LED onboard |
| Còn trống | 34, 5, 15 | 34–39 chỉ input (không làm I2C/XSHUT được); 5, 15 là chân strapping, không dùng cho motor |

⚠ **Không dùng:**
- **GPIO12** (strapping MTDI): bị kéo HIGH lúc reset thì flash nhận 1.8 V, ESP32 không boot và không nạp được. Triệu chứng đã gặp: `boot:0x33` + `invalid header: 0xffffffff`.
- **GPIO34–39 cho I2C hoặc output**: các chân này chỉ là input. **ToF SDA/SCL phải ở 25/26, không nối 34/35.** Chỉ chân GPIO1/INT của ToF (tín hiệu từ cảm biến ra) mới nối được vào 34–39, và firmware không cần chân này.
- 6–11 (flash), 0/1/3 (boot/UART).
- **Chân strapping 0, 2, 15 cho motor**: lúc cấp nguồn GPIO15/0 ở mức HIGH làm motor tự quay khoảng 0.3 s trước khi firmware chạy.

Encoder: cấp **VCC 3.3 V** để tín hiệu ra 3.3 V. Cấp 5 V thì tín hiệu 5 V có thể hỏng ESP32.
GND chung giữa L298N, ESP32, encoder, cảm biến. ESP32 nên cấp bằng buck 5 V riêng. Motor 12 V dùng pin 3S.
Lúc boot, log in kết quả quét từng bus I2C (MPU: `0x68`; ToF: `0x29 0x30`) và trạng thái từng cảm biến.

## Thử motor thô (chẩn đoán dây)
`POST /api/cmd {"op":"motor_test","side":"l"|"r","duty":0.4,"ms":500}`: cấp duty cố định cho 1 bánh (không qua PI, bánh kia phanh, |duty| ≤ 0.8, ≤ 2 s).
Kết quả trong sự kiện `motor_test_done`: tick/mm 2 encoder, góc gyro. Dùng để biết motor nào quay, encoder nào đếm, chiều có khớp không.

## Nạp qua WiFi (OTA) — không cần USB (từ fw 0.2.2)
```bash
pio run -e ota -t upload --upload-port 192.168.1.2      # IP robot (hoặc tworobot-<id>.local)
```
Mật khẩu OTA `tworobot` (trong `platformio.ini` env `ota`). Bắt đầu nạp → firmware **tự E-STOP + phanh motor**.
macOS có thể hỏi cho phép Python nhận kết nối đến (espota mở cổng tạm để robot tải firmware về) → Allow.

## Console (USB serial 115200 hoặc WiFi telnet cổng 23)
`pio device monitor` (USB) hoặc `nc 192.168.1.2 23` (WiFi), gõ:
- `status`: số đo 2 ToF, hướng/bias/nhiệt độ gyro, tick encoder, tốc độ bánh, trạng thái.
- `watch`: bật/tắt in `status` 2 lần/giây.
- Log 10 s/lần và log OTA cũng hiện trên telnet.
- `wifi <ssid> <mật khẩu>`: lưu WiFi rồi khởi động lại (không cần vào AP).
- `reboot`

## Chạy lần đầu (mỗi xe)
1. Nạp firmware. Mở monitor: thấy `AP "TwoRobot-xxxxxx"`.
2. Điện thoại/PC vào WiFi `TwoRobot-xxxxxx` (mật khẩu `tworobot`) → mở `http://192.168.4.1` → nhập WiFi → robot khởi động lại, monitor in IP.
3. Web Control (`tworobot-server`) → **Thiết bị** → Quét subnet (hoặc nhập IP) → gán leader/follower.
4. **Cấu hình**: nhập kích thước thật → **Save flash** (robot mới có cờ `CFG_DEFAULT` và từ chối chạy cho tới khi Save).
5. **Kiểm tra chiều** (kê bánh lên): tab Chạy → jog nhẹ. Bánh quay ngược → bật `motor.invert_l/r`. Xe tiến mà `s` giảm → `enc.invert_l/r`. Quay trái mà góc giảm → `imu.invert_z`.
6. **Hiệu chuẩn** theo thứ tự trong tab Hiệu chuẩn: IMU → Đi thẳng → Quay → ToF quét tường → Khoảng cách ToF.
7. Vẽ đường → **✓ Kiểm tra** → **Start**.

## LED
| Kiểu nháy | Ý nghĩa |
|---|---|
| chớp ngắn 1 lần / 10 s | sẵn sàng |
| chớp 1 / 2 s | IMU chưa calib |
| nháy nhanh liên tục | đang calib IMU |
| nháy 2 Hz | đang chạy |
| sáng liên tục | FAULT / E-STOP |
| nháy dồn dập 2.5 s | Identify (nút trên web) |

## Đã kiểm chứng / chưa
- ✓ Build ESP32 (RAM khoảng 15 %, Flash khoảng 51 %). 12 test native pass, gồm:
  - đối chiếu hàm fit ToF với Python trên 1811 mẫu mô phỏng (lệch < 0.05 mm);
  - **hợp đồng API**: mọi khoá JSON mà server/UI đọc đều có trong phản hồi của bản C++.
- ✓ (2026-11-20) Xe 1 (id 198a2c): nạp OK; I2C thấy 0x68 (MPU), 0x29 + 0x30 (ToF); ToF đo được; gyro tự calib (bias −0.92°/s); vào WiFi; Web Control quét và kết nối (HTTP mỗi request khoảng 55–70 ms). Vòng điều khiển tối đa khoảng 3 ms / 10 ms.
- ✗ **Chưa chạy motor/encoder trên phần cứng**. Cần kiểm tra: chiều motor/encoder, chất lượng PCNT với encoder thật, địa chỉ/clone MPU6050, thời gian vòng `tick_max` (in mỗi 10 s), và hệ số PI (mặc định lấy từ mô phỏng).
- An toàn đấu dây: duty lớn mà bánh quay ngược chiều lệnh > 0.3 s → phanh, báo `WHEEL_DIR_L/R` (motor hoặc encoder đấu ngược).
- Vòng điều khiển trong C++ chưa được mô phỏng vòng kín như bản Python. Bản Python (sim) vẫn là nơi tinh chỉnh thuật toán; sửa thuật toán thì sửa cả hai và chạy lại `gen_vectors.py` + test.

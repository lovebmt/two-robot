# Hardware intake (điền khi có phần cứng)

> **GPIO (§3)** được chép vào `work/firmware/src/board/pins.h` (cố định trong code).
> **Kích thước và thông số (§1–2)** được nhập trên web và lưu flash ([config-management](../../wiki/config-management.md)). File này chỉ để ghi lại số đo gốc và đối chiếu.

Mỗi xe một cột. Nếu 2 xe giống nhau thì chỉ cần ghi xe A.

## 1. Linh kiện
| Mục | Xe A | Xe B |
|---|---|---|
| Board ESP32 (model, số chân) | ESP32-D0WD-V3 (DevKit, USB CP2102), id **198a2c**, MAC 20:50:0d:19:8a:2c | |
| Motor: model / điện áp định mức / rpm | GA25-370 / 12 V / ? rpm | |
| Hộp số: tỉ số | ? | |
| Encoder: PPR (xung/vòng trục motor, 1 kênh) | 11 (cần xác nhận) | |
| Encoder: nguồn 3.3/5 V, đầu ra push-pull/open-collector | | |
| Driver | L298N | L298N |
| Cảm biến khoảng cách: model ×2 | ToF200C (VL53L0X), đã xác nhận | ToF200C |
| Pin (đề xuất) | 3S Li-ion 10.8–12.6 V | |
| ToF200C có chân XSHUT / INT? | | |
| IMU | MPU6050 | MPU6050 |
| Pin: loại, số cell, dung lượng | | |
| Nguồn ESP32: buck riêng / 5V của L298N | | |
| Có LED / buzzer / nút E-STOP? | | |

## 2. Kích thước (mm)
| Mục | Xe A | Xe B |
|---|---|---|
| Thân: dài × rộng × cao | ? × (tạm 167 = 141 + 26, mép ngoài 2 bánh) × ? | |
| Đường kính bánh | **69** (đo 2026-11-20) | |
| Khoảng cách 2 bánh (tâm vệt) | **141** (đo 2026-11-20) | |
| Bề rộng bánh | **26** | |
| Tâm trục bánh tới mặt trước | | |
| Tâm trục bánh tới mặt sau | | |
| Tâm trục tới mắt cảm biến (dọc) | | |
| Khoảng cách 2 mắt cảm biến (ngang), khuyến nghị 80–100 mm | **124** (đo 2026-11-20) | |
| 2 mắt gá chung 1 thanh cứng? Mặt kính cùng mặt phẳng? | | |
| Chiều cao mắt cảm biến so với sàn | | |
| Vị trí MPU6050 (so với tâm trục), chiều trục Z | | |
| Vị trí caster (so với trục) | | |
| Tấm phản xạ đuôi: rộng × cao, chiều cao tâm | | |

Cấu hình đầy đủ đang nạp trên xe A: [robots/robot-198a2c.config.json](robots/robot-198a2c.config.json) (sao lưu từ flash robot).

## 3. GPIO map (đã chốt 2026-11-20 theo dây thật, có sửa 2 chỗ). Nguồn chuẩn: `work/firmware/src/board/pins.h`
| Chức năng | Người dùng nối | Chốt | Lý do sửa |
|---|---|---|---|
| MPU6050 SDA / SCL | 21 / 22 | 21 / 22 | |
| ToF phải: SDA/SCL + XSHUT | 34/35 + 32 | **25/26** + 32 | 34/35 chỉ là input, không làm I2C được → bus I2C riêng 25/26 |
| ToF trái: SDA/SCL + XSHUT | 34/35 + 33 | **25/26** + 33 | như trên |
| Encoder phải A / B | 14 / 27 | 14 / 27 | |
| Encoder trái A / B | 13 / 12 | 13 / **35** | GPIO12 strapping: HIGH lúc reset làm ESP32 không boot (đã thấy `boot:0x33`) |
| L298N ENA / IN1 / IN2 | 15 / 2 / 0 | **23 / 19 / 18** | 15/2/0 là chân strapping: lúc cấp nguồn ENA=HIGH, IN2=HIGH → motor trái tự quay lùi |
| L298N ENB / IN3 / IN4 | 17 / 4 / 16 | 17 / 4 / 16 | giữ nguyên |
| Đo pin | không có | không dùng | xe không có cầu chia |
| LED | | 2 | |

## 4. Ảnh / sơ đồ
Đặt ảnh xe và sơ đồ đi dây vào `work/hardware/img/`.

## Motor (2026-11-21)
- GA25-370 12V **280 RPM**, mua hshop 2026-04-20. Quay tay (0b) đo được hộp số **≈ 1:20.24** (gần chuẩn 1:20.4; không phải 1:34 mặc định), encoder 11 PPR × 4 → **890.5 tick/vòng bánh**.
- Bằng chứng: lệnh đi thẳng 100/1000 mm ra thật 161/1610 mm = 34/21.3.

## Thân xe (2026-11-21)
- Vuông **199 × 199 mm**. Tâm trục bánh → mặt sau/tấm đuôi **68 mm** ⇒ tâm trục → mép trước **131 mm** (sửa 2026-11-21; số 168 trước đó là nhầm).
- Bán kính quét khi quay tại chỗ ≈ **164 mm** (đầu xe văng xa nhất).
- `range.x_mm` = 131: mặt kính ToF sát mép trước (người dùng xác nhận 2026-11-21).

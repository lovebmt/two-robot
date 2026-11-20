# Hardware intake (điền khi có phần cứng)

> **GPIO (§3)** được chép vào `work/firmware/src/board/pins.h` (cố định trong code).
> **Kích thước và thông số (§1–2)** được nhập trên web và lưu flash ([config-management](../../wiki/config-management.md)). File này chỉ để ghi lại số đo gốc và đối chiếu.

Mỗi xe một cột. Nếu 2 xe giống nhau thì chỉ cần ghi xe A.

## 1. Linh kiện
| Mục | Xe A | Xe B |
|---|---|---|
| Board ESP32 (model, số chân) | ESP32 DevKit thường, ? chân | |
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
| Thân: dài × rộng × cao | | |
| Đường kính bánh | | |
| Khoảng cách 2 bánh (tâm vệt) | | |
| Tâm trục bánh tới mặt trước | | |
| Tâm trục bánh tới mặt sau | | |
| Tâm trục tới mắt cảm biến (dọc) | | |
| Khoảng cách 2 mắt cảm biến (ngang) | | |
| Chiều cao mắt cảm biến so với sàn | | |
| Vị trí MPU6050 (so với tâm trục), chiều trục Z | | |
| Vị trí caster (so với trục) | | |
| Tấm phản xạ đuôi: rộng × cao, chiều cao tâm | | |

## 3. GPIO map (đề xuất, sửa theo thực tế)
Xem ràng buộc chân ở [architecture §2.4](../../wiki/architecture.md#24-ràng-buộc-gpio-của-esp32-để-chọn-chân-khi-có-phần-cứng).

| Chức năng | Gợi ý | Thực tế |
|---|---|---|
| L298N ENA (PWM trái) | 25 | |
| L298N IN1 / IN2 | 26 / 27 | |
| L298N ENB (PWM phải) | 14 | |
| L298N IN3 / IN4 | 23 / 5 | |
| Encoder trái A / B | 34 / 35 (cần pull-up ngoài) | |
| Encoder phải A / B | 36 / 39 (cần pull-up ngoài) | |
| I²C bus 0 SDA / SCL (2× ToF200C) | 21 / 22 | |
| ToF XSHUT trái / phải | 18 / 19 | |
| ToF INT trái / phải (tuỳ chọn) | 16 / 17 | |
| I²C bus 1 SDA / SCL (MPU6050) | 13 / 4 *(hoặc dùng chung bus 0)* | |
| MPU6050 INT (tuỳ chọn) | 33 | |
| ADC pin (qua cầu chia, ADC1) | 32 | |
| LED trạng thái | 2 (LED onboard) | |
| Buzzer (tuỳ chọn, qua transistor) | 15 | |

Ghi chú: chân 5 và 15 là strapping nhưng dùng làm output sau khi boot thì được. Chân 12 không dùng. Chân 0 để dành cho nút BOOT.

## 4. Ảnh / sơ đồ
Đặt ảnh xe và sơ đồ đi dây vào `work/hardware/img/`.

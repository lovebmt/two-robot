# Câu hỏi mở

## Đã chốt (2026-11-20, nguồn: [đợt 1](../raw/2026-11-20-answers-1.md), [đợt 2](../raw/2026-11-20-answers-2.md))
| # | Câu hỏi | Kết luận |
|---|---|---|
| Q1 | Web trên PC hay ESP32 | **PC** |
| Q3 | Liên lạc giữa 2 xe | **Qua PC**, không có ESP-NOW |
| Q4 | Loại cảm biến khoảng cách | **ToF200C** ×2 (nhân VL53L0X, I²C) |
| Q5 | IMU | **Có MPU6050** (đợt 2 sửa lại đợt 1) |
| Q7 | Loại góc cua | **Chỉ 90°**, không vẽ được góc khác |
| Q8 | Cách cua | Quay tại chỗ, stop-and-go |
| Q11 | Phần cứng | ESP32, L298N, GA25-370 có encoder. **GPIO cung cấp sau** |
| Q19 | Board ESP32 | ESP32 thường (WROOM DevKit) |

## Còn mở (cần trước M4, phần cứng)
| # | Câu hỏi | Giả định đang dùng |
|---|---|---|
| Q15 | GA25-370: **bản 6 V hay 12 V, tỉ số hộp số / rpm**? Encoder cấp 3.3 hay 5 V? | 11 PPR; tỉ số chưa biết |
| Q16 | Pin và nguồn (2S hay 3S? buck riêng cho ESP32?) | 3S nếu motor 12 V, 2S nếu 6 V. Buck 5 V riêng |
| Q17 | Kích thước thật: thân xe, bánh, track, vị trí cảm biến và IMU | Điền [intake](../work/hardware/hardware-intake.md) |
| Q18 | Có làm được tấm phản xạ phẳng ở đuôi xe không? | Có |
| Q21 | Module ToF200C có chân XSHUT và INT không (sơ đồ chân thực tế)? | Có XSHUT. INT tuỳ chọn |
| Q22 | Board DevKit 30 hay 38 chân? | 30 chân |
| GPIO | Bảng GPIO thực tế | Đề xuất ở intake §3 |

## Còn mở (không chặn, có giả định)
| # | Câu hỏi | Giả định |
|---|---|---|
| Q2 | Cách nạp WiFi cho xe | AP + trang cấu hình khi chưa có WiFi |
| Q9 | Gap định nghĩa thế nào | Lưu tâm–tâm. UI cho nhập "khe hở mũi–đuôi" |
| Q10 | Tốc độ, kích thước sân, ô lưới | 150 mm/s, 3×2 m, ô 100 mm |
| Q12 | Đường khép kín, lặp nhiều vòng | Hỗ trợ khép kín, chạy 1 lần |
| Q13 | Ngôn ngữ server | Python FastAPI |
| Q14 | Đảo vai trò, chạy lùi | Không ở v1 |
| Q20 | Router cho sân test (tắt được client isolation không)? | Có router riêng |

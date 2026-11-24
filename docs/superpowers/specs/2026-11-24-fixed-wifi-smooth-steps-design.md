# WiFi cố định và chuyển động step mượt

## Mục tiêu

Bỏ AP cài WiFi khỏi firmware và làm mượt tăng/giảm tốc trong từng bước của
stop-and-go, không thay đổi trình tự cua hiện tại.

## WiFi

- Bỏ `WIFI_AP`, form `/wifi` và mọi trạng thái AP.
- WiFi STA dùng `WIFI_SSID/WIFI_PASS` trong `secrets.h` khi có.
- Nếu không có credentials biên dịch, dùng NVS đã lưu.
- Giữ console USB `wifi <ssid> <pass>` làm cứu hộ; lệnh này ghi NVS và reboot.
- Không có WiFi thì xe chạy offline, không phát AP.

## Chuyển động

- Giữ stop-and-go và primitive hiện tại.
- Thêm bộ giới hạn jerk dùng chung cho vận tốc:

```text
a_des = clamp((v_target - v) / dt, -a_max, a_max)
jerk  = 4 × a_max
a_new = clamp(a_des, a_old - jerk·dt, a_old + jerk·dt)
v_new = v + a_new·dt
```

- `DRIVE_TO` dùng profile này cho tăng và giảm tốc tuyến tính.
- `TURN` dùng cùng profile cho vận tốc góc, với giới hạn từ
  `motion.turn_acc_dps2`.
- `FOLLOW` dùng profile cho vận tốc tiến; điều kiện dừng an toàn vẫn phanh ngay.
- Không thêm tham số schema/UI.

## Kiểm thử

- Unit test C++ và Python cho ramp: gia tốc thay đổi mỗi mẫu không vượt jerk.
- Chạy lại toàn bộ test native/Python/sim để bảo đảm stop-and-go vẫn hoàn thành
  đường L, rect và zigzag.
- Build ESP32, nạp OTA/USB và xác nhận log không còn AP.

## Tiêu chí hoàn thành

- Không còn chuỗi AP/192.168.4.1 khi có credentials WiFi.
- Đường stop-and-go vẫn hoàn thành; sai số góc không tăng.
- Không còn bước nhảy gia tốc ở thời điểm primitive bắt đầu hoặc bắt đầu giảm tốc.

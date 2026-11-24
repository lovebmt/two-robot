# Dừng hệ thống khi một hoặc hai mắt ToF bất thường

## Mục tiêu

Trong chế độ 2 xe, dừng follower ngay tại chỗ khi một hoặc cả hai mắt ToF của follower
đột ngột báo gần hơn hoặc mất tín hiệu. PC phải dừng leader, chuyển coordinator sang
`FAULT` và hiển thị cảnh báo nổi bật trên Web UI.

## Phạm vi

- Chỉ áp dụng khi follower đang chạy primitive `FOLLOW` của chế độ 2 xe.
- Không đổi chế độ 1 xe, bám tấm, planner hoặc đường đi.
- Bắt buộc nạp firmware mới vì follower phải phanh cục bộ trước khi PC phản ứng.

## Phát hiện bất thường

Guard nhận trực tiếp cặp số đo raw ToF mỗi mẫu, không dùng giá trị đã lọc trung vị.
Mỗi mắt có lịch sử 5 mẫu hợp lệ gần nhất.

Một mắt được coi là bất thường khi:

- Đã có ít nhất 3 mẫu hợp lệ trước đó và giá trị hiện tại ngắn hơn trung vị lịch sử
  ít nhất `safety.tof_anomaly_jump_mm` (mặc định 120 mm).
- Mẫu hiện tại không hợp lệ sau khi mắt đã có baseline hợp lệ.

Một hoặc hai mắt bất thường trong `safety.tof_anomaly_samples` mẫu liên tiếp
(mặc định 2 mẫu) đều kích hoạt fault `OBSTACLE_INTRUSION`.

Guard chỉ armed sau khi cả hai mắt từng có ít nhất 3 mẫu hợp lệ trong chế độ
`FOLLOW`, tránh báo nhầm ngay khi primitive vừa bắt đầu.

## Phản ứng

1. Firmware follower phanh motor và chuyển `FAULT` với mã `OBSTACLE_INTRUSION`.
2. Firmware phát event `fault` qua hộp thư telemetry.
3. Coordinator nhận event, ghi lỗi và phanh leader trong vòng một chu kỳ poll.
4. Coordinator chuyển trạng thái `FAULT`; hai xe không tự chạy tiếp.
5. Web UI hiện toast đỏ toàn trang và ghi tab Sự kiện. Thông báo ghi rõ
   “Vật cản/bất thường ToF giữa 2 xe; kiểm tra hai mắt ToF”.
6. Chỉ được chạy lại sau khi xóa vật cản, bấm `Reset lỗi` rồi chạy lại.

## Tham số mới

- `safety.tof_anomaly_jump_mm`: float, mm, default `120`, min `20`, max `600`, live.
- `safety.tof_anomaly_samples`: int, số mẫu, default `2`, min `1`, max `10`, live.

Hai tham số thuộc nhóm `safety`; cập nhật schema làm nguồn chung cho C++ và Python.

## Kiểm thử

- Guard không báo khi khoảng cách thay đổi từ từ dưới ngưỡng.
- Guard không báo trước khi arming đủ baseline.
- Một mắt giảm đột ngột 120 mm hai mẫu liên tiếp → bất thường.
- Hai mắt cùng giảm → bất thường.
- Một mắt mất tín hiệu hai mẫu; hai mắt cùng mất → bất thường.
- Firmware `App` báo `OBSTACLE_INTRUSION` chỉ khi đang chạy `FOLLOW`.
- Coordinator nhận fault và đưa `FAULT` lên UI.
- Chạy test Python, test C++ native, build ESP32.

## Tiêu chí hoàn thành

- Vật 15 mm che một mắt ToF trong mô phỏng làm dừng cả hai xe.
- Leader không tiếp tục chạy quá một chu kỳ coordinator sau fault.
- UI hiển thị cảnh báo rõ và event log có `OBSTACLE_INTRUSION`.
- Firmware mới build và nạp OTA thành công lên xe thật.

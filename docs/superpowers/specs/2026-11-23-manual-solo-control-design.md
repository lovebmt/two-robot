# Điều khiển tay cho chế độ 1 xe

## Mục tiêu

Thêm bảng điều khiển tay trên Web Control để người dùng điều khiển một xe tiến,
lùi, quay trái, quay phải và dừng. Bảng điều khiển chỉ có ở chế độ `solo` (1 xe),
sử dụng robot đang được chọn ở bộ chọn xe.

## Phạm vi

- Chỉ hiện card điều khiển khi `store.mode === "solo"`.
- Không hiện ở chế độ 2 xe hoặc chế độ bám tấm.
- Không thay đổi planner lưới và API chạy đường hiện tại.
- Không thêm vẽ tự do, đường cong hoặc primitive mới trong firmware.

## Giao diện

Đặt card **Điều khiển tay** ở đầu cột phải tab Chạy, trước card Điều phối.

- D-pad gồm Tiến, Lùi, Trái, Phải.
- Nút STOP nằm giữa D-pad.
- Tiến/lùi dùng `v = ±80 mm/s`, `w = 0`.
- Trái/phải dùng `v = 0`, `w = ±0.8 rad/s` (khoảng ±45.8°/s).
- Hiển thị trạng thái `Sẵn sàng`, `Đang tiến`, `Đang lùi`, `Đang quay trái`,
  `Đang quay phải`, `Đang dừng` hoặc `Không khả dụng`.
- Card bị ẩn hoàn toàn ngoài chế độ 1 xe.
- Nút bị vô hiệu hoá khi chưa có xe được chọn, xe offline hoặc xe không ở
  trạng thái `IDLE`/`JOG`.

## Hành vi

- Giữ nút bằng chuột hoặc cảm ứng bắt đầu gửi lệnh jog ngay lập tức.
- Trong lúc giữ, Web gửi lại lệnh jog mỗi 100 ms. Firmware tự hết hạn lệnh jog
  sau 300 ms nên không tiếp tục chạy nếu heartbeat bị mất.
- Nhả nút, `pointercancel`, `lostpointercapture`, `window.blur` hoặc chuyển tab
  ẩn khỏi trang phải gửi lệnh stop ngay.
- Nút STOP và mọi lần kết thúc điều khiển phải gửi lệnh stop riêng để firmware
  trở về `IDLE`, sau đó mới cho phép chạy đường.
- Chỉ một hướng được giữ tại một thời điểm. Nhấn hướng khác trong lúc đang giữ
  phải dừng lệnh cũ trước khi bắt đầu lệnh mới.
- Lỗi gửi lệnh phải dừng lặp lệnh, gọi stop nếu còn kết nối và hiển thị toast lỗi.

## API

### Jog

Giữ endpoint hiện có:

```text
POST /api/robots/{rid}/jog
{"v": 80, "w": 0}
```

Server phải từ chối giá trị không hữu hạn và giới hạn phạm vi trước khi gửi:

- `-150 <= v <= 150` mm/s.
- `-2 <= w <= 2` rad/s.

### Stop

Thêm endpoint:

```text
POST /api/robots/{rid}/jog/stop
```

Endpoint gửi lệnh firmware `{"t": "cmd", "op": "stop"}` tới xe được chọn và
trả `{"ok": true}` khi lệnh đã được gửi.

## Đồng bộ trạng thái

- Một module frontend riêng quản lý interval, pointer capture và lệnh đang giữ.
- `main.js` cập nhật class `mode-solo` trên `body` khi đổi chế độ.
- Khi đổi robot, đổi chế độ hoặc tab, lệnh điều khiển tay đang giữ phải dừng.
- Telemetry từ frame hiện có cung cấp trạng thái và tư thế xe.

## Kiểm thử

- Unit test server: clamp jog, từ chối `NaN`/`Infinity` và gửi stop đúng lệnh.
- Kiểm thử giao diện với robot ảo: card ẩn ở `pair`/`follow`, hiện ở `solo`.
- Giữ Tiến gửi lặp; nhả gửi stop; STOP dừng ngay.
- `window.blur` và đổi chế độ dừng lệnh.
- Sau STOP, chạy thử một đường solo ngắn để xác nhận xe không còn bị kẹt ở `JOG`.
- Chạy toàn bộ test server hiện có.

## Tiêu chí hoàn thành

- Chế độ 2 xe và bám tấm không thay đổi hành vi.
- Chế độ 1 xe điều khiển được xe thật bằng bốn nút và STOP.
- Nhả nút hoặc mất kết nối không để xe tiếp tục chạy.
- Không cần nạp lại firmware cho tính năng này.

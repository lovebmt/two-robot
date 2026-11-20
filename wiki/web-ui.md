# Web Control: màn hình và hành vi

Frontend Vite + TS + Canvas, giao tiếp với server qua REST và WS. Mỗi lần dùng là một project JSON (mở/lưu).

## 1. Màn hình

### Devices
- Nút **Scan** (đồng thời luôn nghe beacon). Bảng xe: tên, IP, MAC, fw, pin, RSSI, ping (ms), trạng thái online.
- **Identify**: xe nháy LED và lắc bánh. Hai ô chọn **Leader / Follower**, không cho 2 xe trùng vai trò.
- Nút kết nối và ngắt kết nối. Có cảnh báo khi fw 2 xe khác nhau.

### Config
- Hai tab, mỗi xe một tab, có nút "Copy sang xe kia". **Form sinh tự động từ schema** ([config-management.md](config-management.md)), theo nhóm trong [parameters.md](parameters.md). Có nút Apply / Save / Revert / Diff / Export / Import / Factory reset. Ô chưa lưu có viền vàng, ô khác với xe có viền tím. Nhóm `hw.*` có nút Reboot.
- **Hình xem trước xe (vẽ từ tham số):** thân hộp, 2 bánh, caster, 2 mắt cảm biến và chùm của chúng, tấm phản xạ, vòng quét `R_sw`. Hình cập nhật khi đổi số.
- Hiển thị các giá trị suy ra: `mm_per_tick`, `R_sw`, `g_hold_min`, `exit_dist_min`, `seg_min`.
- Nút **Push to robot** (PUT `/api/config`) và **Pull**. Có so sánh khác biệt giữa bản trên PC và bản trên xe.

### Map & Path
- Lưới `cols × rows`, ô `cell_mm`, có thước đo theo cm. Zoom và pan.
- Công cụ: **Start** (click đỉnh, rồi click hướng), **Draw** (click lần lượt các đỉnh), Move point, Delete point, Clear, Undo/Redo.
- Ràng buộc khi vẽ: điểm mới luôn được ép cùng hàng hoặc cùng cột với điểm trước. Nếu tạo góc 180° (quay ngược lại) thì không cho vẽ. Điểm thẳng hàng tự gộp lại.
- Overlay: số thứ tự góc cua, mũi tên rẽ trái/phải, vòng tròn `R_sw` tại mỗi góc, vị trí follower lúc xuất phát, đoạn vi phạm tô đỏ.
- Panel thông tin: tổng chiều dài, số góc, thời gian ước tính, kết quả validator.

### Simulate
- Khởi động hoặc dừng sim process, chọn kịch bản YAML và seed. Robot ảo xuất hiện ở màn Devices như xe thật.
- Thanh trượt vật lý (lệch motor, μ, nhiễu, trễ mạng), vẽ vùng ma sát lên lưới, nút gây lỗi.
- Chạy batch nhiều seed và xem bảng metric pass/fail.

### Run (view chung cho SIM / REAL / REPLAY, xem [simulation §5](simulation.md#5-view-một-giao-diện-cho-cả-3-nguồn))
- Badge nguồn `SIM` / `REAL` / `REPLAY`. Ở sim có thêm lớp truth và điều khiển thời gian (pause, step, ×0.25–×4). Ở replay có timeline kéo được.
- Nút lớn: **Load path**, **Arm**, **Start**, **Pause/Resume**, **Stop**, **E-STOP** (đỏ, phím tắt `Space`).
- Lưới realtime với pose odometry của 2 xe và giá trị 2 cảm biến (vẽ thành tia).
- **Corner stepper**: danh sách 10 bước của góc hiện tại, đánh dấu bước đã xong, đang chạy, lỗi. Khi lỗi có nút Retry / Jog / Abort.
- Đồ thị: gap (ước lượng, đo, mục tiêu), `v_L`/`v_F`, `Δψ`. Có nút Record log và download.
- Chế độ **Jog**: joystick hoặc phím WASD cho xe đang chọn.

### Calibrate
Wizard cho từng bài trong [algorithms §6](algorithms.md#6-hiệu-chuẩn) và [motor-control §3](motor-control.md#3-hiệu-chuẩn-trên-web-wizard). Mỗi bài: hướng dẫn, Run, nhập số đo thực, xem biểu đồ và giá trị đề xuất, rồi Apply và Save vào flash.
Có thêm nút "Xuất mô hình motor sang sim" để đưa `K`, `τ`, LUT của xe thật vào kịch bản sim.

## 2. Planner (nét vẽ → đường đi)
1. Danh sách đỉnh lưới `P0..Pn` được đổi sang mm (`× cell_mm`, gốc toạ độ ở `P0`, hướng start là trục x).
2. Mỗi cặp đỉnh thành một segment `len`. Mỗi đỉnh giữa `turn_after` = dấu của tích có hướng (±90).
3. Tính `corners_s` và `follower.start_s = −gap.target`.
4. CRC32 trên JSON chuẩn hoá. Kết quả: `path_id` tăng dần.

## 3. Validator
| Quy tắc | Lỗi/cảnh báo |
|---|---|
| ≥ 1 segment, chỉ ngang/dọc, góc ∈ {±90} | lỗi |
| Không tự cắt và không đè lên đoạn khác | lỗi |
| Mỗi segment sau góc ≥ `exit_dist` | lỗi |
| Segment đầu ≥ `approach_dist` | cảnh báo |
| `g_hold ≥ g_hold_min`, `exit_dist ≥ exit_dist_min` | lỗi |
| `gap.target ≥ gap.min_safe` và nằm trong vùng đo cảm biến | lỗi |
| `v_cruise ≤ headroom·min(v_max leader, v_max follower)` | cảnh báo |
| Follower start (lùi `gap` sau P0) nằm trong bản đồ | lỗi |
| Đường nằm trong bản đồ, cách biên ≥ nửa chiều rộng xe | cảnh báo |
| Vòng `R_sw` tại góc không chạm đoạn khác | cảnh báo |

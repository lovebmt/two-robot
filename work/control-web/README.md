# Web Control + mô phỏng 2 robot

Server Python (FastAPI) và giao diện web (JS module thuần, không cần build) để:
- bật **2 robot ảo** chạy "firmware ảo" trong mô hình vật lý,
- gán vai trò đầu/đuôi, chỉnh tham số (RAM/flash), vẽ đường trên lưới,
- chạy và xem realtime trên **một view chung** cho SIM / REAL / REPLAY, ghi log và phát lại.

## Chạy

```bash
cd work/control-web
uv venv --python 3.12 .venv && uv pip install -e ".[dev]"   # lần đầu
.venv/bin/tworobot-server                                   # http://127.0.0.1:8080
```

Thứ tự thao tác trên UI:
1. **Thiết bị** → "Bật 2 robot ảo". Robot A là leader, B là follower; có thể đổi vai trò.
2. **Bản đồ & Đường** → chọn "Mẫu…" hoặc click các đỉnh lưới để vẽ. Panel bên phải báo lỗi và cảnh báo.
3. **Chạy đường này ▶** → tab **Chạy / Xem**: 2 xe, tia ToF, vết đi, bảng 10 bước ở góc cua, đồ thị gap / tốc độ bánh / góc.
4. **Cấu hình** → sửa tham số. Apply chỉ ghi RAM, Save ghi flash (file `.simstate/<id>/config.json` với robot ảo).
5. Panel **Mô phỏng**: dừng/chạy, tốc độ ×0.25–×8, gây lỗi (che ToF, đẩy xe, mất link, caster kẹt), thanh trượt vật lý (lệch motor, ma sát, trượt, bias gyro, nhiễu ToF), ghi log và replay.

Có thể mở thẳng một tab bằng URL: `/#map`, `/#run`, `/#config`, `/#calib`, `/#devices`.

**Trước khi chạy xe thật:** tab **Hiệu chuẩn** cho từng xe: IMU → Đi thẳng → Quay → ToF quét tường → Khoảng cách ToF.
Ở tab Chạy, bấm **✓ Kiểm tra** để xem xe đã đặt đúng chưa. Start cũng tự kiểm tra và từ chối chạy nếu không đạt.
Robot ảo: ô "Đặt xe lệch ± mm / ± °" trong khung Mô phỏng giả lập người đặt xe không chuẩn.

## Robot thật: HTTP qua LAN
Robot thật (ESP32) giao tiếp với server **chỉ bằng HTTP**: lệnh `POST /api/cmd`, trạng thái `GET /api/state` hỏi 20 Hz, kèm hộp thư sự kiện. Đặc tả đầy đủ ở [wiki/protocol.md](../../wiki/protocol.md).
Tab **Thiết bị** → ô "Robot thật" → nhập subnet (mặc định subnet của PC), IP hoặc IP:cổng → **Quét**.

Thử ngay khi chưa có ESP32, bằng **robot HTTP ảo** (mỗi robot là một HTTP server thật, cùng API với ESP32):
```bash
.venv/bin/tworobot-vrobot --n 2 --port 8101      # terminal 1
.venv/bin/tworobot-server                        # terminal 2 → Thiết bị → Quét "127.0.0.1:8101-8102"
```
Robot HTTP ảo tự đặt vào vị trí xuất phát khi nhận đường (thay cho người đặt xe), và gửi kèm vị trí thật để view so sánh.

## Chạy không UI (hồi quy / chấm điểm)

```bash
.venv/bin/tworobot-batch --path rect --seeds 3
.venv/bin/tworobot-batch --path zigzag --mismatch '{"leader":{"motor_scale_r":0.85},"follower":{"slip_turn":0.06}}'
.venv/bin/python -m pytest -q
```
Metric: hoàn thành, va chạm, gap RMSE trên đoạn thẳng, gap tâm–tâm nhỏ nhất, sai số vị trí cua (follower so với chỗ leader thực sự quay).

## Cấu trúc

```
server/tworobot/
  params.py        schema chung (../shared/config.schema.json), validate, giá trị suy ra, đội hình
  fw/              FIRMWARE ẢO — bản tham chiếu sẽ port sang C++ (lib/core, lib/app)
    control.py       PI bánh + FF + bão hoà ưu tiên ω + học FF; fusion gyro/encoder + ZUPT; odometry
    gapkf.py         Kalman filter gap [g, v_L]
    rangeproc.py     2 ToF → ρ (gap tâm–tâm), Δψ, mép tấm
    primitives.py    DRIVE_TO, TURN, HOLD, FOLLOW, SNAPSHOT, ALIGN, REACQUIRE
    paramsvc.py      tham số động: validate → derive → swap ở đầu chu kỳ
    robot.py         máy trạng thái, an toàn, HTTP/WS handler, telemetry 25 Hz
    hal.py           interface phần cứng
  sim/             world.py (vật lý, gyro MPU6050, ToF ray-cast, va chạm), engine.py, batch.py, vrobot.py (robot HTTP ảo)
  links.py         SimLink (robot ảo trong process) | NetLink (robot thật, HTTP poll 20 Hz)
  discovery.py     quét HTTP GET /api/info (subnet, IP, IP:cổng, dải cổng)
  registry.py      danh sách robot, vai trò, relay leader→follower
  coordinator.py   quy trình 10 bước ở góc cua
  planner.py       nét vẽ → segments, validator
  app.py           FastAPI + WS /ws/ui
web/               index.html, css/, js/ (main, devices, config, mapedit, view, charts, draw, api, store)
tests/             params, planner, kịch bản sim
```

## Giới hạn hiện tại
- Chưa có firmware ESP32. Phía PC (NetLink HTTP) đã được kiểm chứng qua socket thật bằng robot HTTP ảo (`tests/test_http.py`).
- Robot ảo "trong server" (nút Bật 2 robot ảo) nối qua gọi hàm (SimLink) để chạy nhanh và deterministic. Muốn kiểm tra đúng đường HTTP thì dùng `tworobot-vrobot`.
- Không cho vẽ đường khép kín (theo yêu cầu): trình vẽ chặn, validator báo lỗi.

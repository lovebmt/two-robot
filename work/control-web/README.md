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

Có thể mở thẳng một tab bằng URL: `/#map`, `/#run`, `/#config`, `/#devices`.

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
  sim/             world.py (vật lý, ToF ray-cast, va chạm), engine.py (bước 2 ms, lập lịch task), batch.py
  links.py         SimLink (robot ảo trong process) | NetLink (ESP32 — M4)
  registry.py      danh sách robot, vai trò, relay leader→follower
  coordinator.py   quy trình 10 bước ở góc cua
  planner.py       nét vẽ → segments, validator
  app.py           FastAPI + WS /ws/ui
web/               index.html, css/, js/ (main, devices, config, mapedit, view, charts, draw, api, store)
tests/             params, planner, kịch bản sim
```

## Giới hạn hiện tại
- **NetLink (robot thật) chưa có**: server đã nghe beacon UDP 47800 và hiện robot thật trong danh sách, nhưng chưa điều khiển được. Phần này làm cùng firmware (M4).
- Robot ảo nói chuyện với server qua gọi hàm trong process, nhưng message vẫn đúng định dạng JSON của giao thức. Lớp HTTP/WS/UDP thật sẽ được kiểm chứng khi có firmware.
- Đường khép kín (quay về điểm đầu) đang bị validator coi là tự cắt.

# Giao thức truyền thông: HTTP qua LAN

Nguồn: [đợt 7](../raw/2026-11-20-answers-7.md). Quyết định D25: **robot thật giao tiếp với PC hoàn toàn bằng HTTP/1.1 (JSON) qua LAN.**
Không dùng WebSocket, không dùng UDP. 2 robot không nói chuyện trực tiếp; mọi thứ đi qua PC (C2).

ESP32 chạy HTTP server ở cổng 80. PC là client:
- **Lệnh** gửi bằng `POST`, robot trả ack/nack ngay trong response.
- **Trạng thái** PC hỏi bằng `GET /api/state` 20 lần/giây (polling). Mỗi lần trả về telemetry mới nhất và các sự kiện chưa lấy.
- **Heartbeat** chính là các request đó: robot không nhận request nào quá `safety.link_timeout_ms` (500 ms) khi đang ARM thì phanh và báo `LINK_LOST`.

Trình duyệt ⇄ PC server vẫn dùng WebSocket `/ws/ui` (nội bộ máy tính, không liên quan robot).

Tham chiếu cài đặt: `work/control-web/server/tworobot/fw/robot.py` (`handle_request`), robot HTTP ảo `tworobot-vrobot`, phía PC là `links.py` (`NetLink`).

## 1. Tìm robot (discovery)
PC quét `GET http://<ip>/api/info` (timeout 0.4 s, 64 request song song). Mục tiêu nhập được:
`192.168.1.0/24` (cả subnet, mặc định là subnet của PC), `192.168.1.42`, `192.168.1.42:8080`, hoặc `127.0.0.1:8101-8102` (dải cổng).
Robot nhận dạng qua `/api/info` có `id` và `fw`. Nên đặt IP tĩnh (DHCP reservation) cho 2 xe để lần sau quét nhanh hoặc nhập thẳng IP. mDNS (`tworobot-xxxx.local`) là tuỳ chọn.

## 2. HTTP API của robot
Mọi body là JSON. Mã 200 = OK, 400 = tham số sai, 409 = trạng thái không cho phép, 404 = không có route.

| Method | Path | Body / query | Trả về |
|---|---|---|---|
| GET | `/api/info` | | `{id, name, fw, role, state, cfg_rev, schema_ver}` |
| GET | `/api/state` | `?since=<eseq>` | `{tel, events[], eseq, events_lost}` (§3) |
| POST | `/api/cmd` | `{op, ...}` (§4) | `{ok, err?, state}` (200 nếu ack, 409 nếu nack) |
| POST | `/api/relay` | `{t_ms, s, v, phase}` | `{ok}`: trạng thái leader do PC chuyển cho follower |
| GET | `/api/config` | | `{params, derived, cfg_rev, crc, saved_crc, dirty, flags}` |
| PUT | `/api/config` | `{patch: {key: value}, persist: bool}` | `{ok, cfg_rev, changed[]}` hoặc `{ok:false, errors:{key:msg}, busy?}` |
| POST | `/api/config/save` · `/reload` · `/factory_reset` | | `{ok, cfg_rev, crc}` |
| PUT | `/api/role` | `{role: "leader"\|"follower"\|null}` | `{ok, role}` (lưu NVS) |
| PUT | `/api/path` | đường đi (§6) | `{ok, path_id, crc}` |
| POST | `/api/identify` | | `{ok}`: nháy LED và lắc bánh 2.5 s |

### 2.1 Điều khiển tay qua Control Server

Web Control giữ nút điều khiển bằng cách gửi lặp lệnh jog cho tới khi nhả nút. Server giới hạn
`v ∈ [-150, 150]` mm/s và `w ∈ [-2, 2]` rad/s.

| Method | Path | Body | Trả về |
|---|---|---|---|
| POST | `/api/robots/{rid}/jog` | `{v, w}` | `{ok}` |
| POST | `/api/robots/{rid}/jog/stop` | | `{ok}` |

Server chuyển hai request này thành lệnh robot `/api/cmd` với `op: "jog"` hoặc `op: "stop"`.
Lệnh jog trên robot tự hết hạn sau 300 ms; UI gửi lặp mỗi 100 ms trong lúc giữ nút.

## 3. `GET /api/state?since=N`
```json
{ "tel": { "t_ms": 123450, "state": "RUNNING", "role": "follower", "cfg_rev": 4, "fault": null,
           "est": {"x":..,"y":..,"yaw":..,"s":..,"v":..,"w":..},
           "wheels": {"l": {"v_cmd":..,"v":..,"duty":..,"adapt":..}, "r": {...}},
           "tof": {"l": {"mm": 402, "st": 0}, "r": {"mm": 399, "st": 0}},
           "gap": {"est":..,"meas":..,"target":..,"vL":..}, "dpsi": 0.004,
           "imu": {"ok": true, "bias_dps":.., "calib": null, "temp_c":.., "k_dps_c":.., "zupt": false},
           "prim": {"id": 73, "name": "FOLLOW"}, "flags": ["SAT"], "vbat": 11.9, "geom": {...} },
  "events": [ {"t":"evt","e":"prim_done","id":72,"prim":"TURN","ok":true,"result":{...},"eseq":41},
              {"t":"evt","e":"calib_done","ok":true,"result":{"bias_dps":1.2,...},"eseq":42} ],
  "eseq": 42, "events_lost": false }
```
- Robot giữ **64 sự kiện** gần nhất trong hộp thư, mỗi sự kiện có số `eseq` tăng dần. PC gửi `since` = `eseq` lớn nhất đã nhận, nên không mất sự kiện nếu một lần poll bị lỗi. `events_lost=true` nghĩa là PC tụt quá 64 sự kiện.
- Nếu `eseq` trả về nhỏ hơn `since` thì robot đã khởi động lại, PC đếm lại từ 0.
- Các loại sự kiện: `prim_done`, `calib_done`, `fault` (`code`: `LEADER_LOST`, `LINK_LOST`, `BAT_LOW`, ...).
- Kích thước khoảng 1.5–2 KB mỗi lần × 20 Hz × 2 xe ≈ 80 KB/s, rất nhỏ so với WiFi.

## 4. `POST /api/cmd`
| `op` | Tham số | Ghi chú |
|---|---|---|
| `arm` | `path_id` | cần: đã có path, có vai trò, không còn `CFG_DEFAULT`, IMU đã calib |
| `prim` | `id`, `prim`, các tham số primitive | `DRIVE_TO{s_end, heading_deg?, v_max?}`, `TURN{deg \| heading_deg}`, `FOLLOW{g_target, gap_min_safe, s_limit?}`, `SNAPSHOT{n?}`, `ALIGN{tol_deg?}`, `REACQUIRE{rho_exp, tol?, scan_deg?}`, `HOLD`. Kết quả về qua sự kiện `prim_done` |
| `calib_imu` | | xe phải đứng yên `imu.calib_time_s`. Kết quả về qua `calib_done` |
| `pause` / `resume` / `stop` | | |
| `estop` / `reset` | | `reset` xoá FAULT/ESTOP |
| `set_s` | `s` | đặt lại tiến độ (neo ở góc cua) |
| `jog` | `v`, `w` | chạy tay, hết hạn sau 300 ms (PC gửi lặp lại) |

## 5. Thời gian và độ trễ
- Trong LAN, mỗi request mất khoảng 2–20 ms (đo được 2–3 ms với robot HTTP ảo trên localhost). Server hiển thị RTT từng robot ở tab Thiết bị.
- Vòng điều khiển nhanh nằm trên xe, nên độ trễ HTTP chỉ ảnh hưởng tới các bước ở góc cua (chậm, tuần tự) và relay `v_L` (phụ trợ, follower bám bằng ToF).
- ESP32: dùng HTTP keep-alive, `WiFi.setSleep(false)`. Handler phải ngắn và không chặn task điều khiển: đọc snapshot telemetry đã chuẩn bị sẵn, không tính toán trong handler.

## 6. Định dạng đường đi (`PUT /api/path`)
```json
{ "path_id": 12, "units": "mm",
  "start": {"x": 500, "y": 300, "yaw_deg": 0},
  "segments": [ {"len": 1600, "heading_deg": 0,  "turn_after": 90, "a": [500,300],  "b": [2100,300]},
                {"len": 1100, "heading_deg": 90, "turn_after": 0,  "a": [2100,300], "b": [2100,1400]} ],
  "corners_s": [1600], "total_len": 2700,
  "follower": {"start_s": -320, "start": {"x": 180, "y": 300, "yaw_deg": 0}},
  "formation": {"g_target": 320, "g_hold_min": 284, "exit_dist": 240, "gap_min_safe": 175, "leader_rear_mm": 40, ...},
  "crc32": "8f3a12c0" }
```
`turn_after` ∈ {+90, −90, 0}, dương là rẽ trái. Đường luôn hở: không khép kín, không tự cắt, không quay đầu 180°.

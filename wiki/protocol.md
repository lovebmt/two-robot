# Giao thức truyền thông

Tất cả đi qua PC (ràng buộc C2). Ba kênh:

| Kênh | Hướng | Dùng cho | Đặc tính |
|---|---|---|---|
| UDP `47800` broadcast | xe → LAN | beacon discovery 1 Hz | best-effort |
| HTTP `:80/api/*` | PC → xe | info, config, path, calib (payload lớn) | tin cậy |
| WebSocket `:80/ws` | PC ⇄ xe | lệnh, ACK, sự kiện, heartbeat | tin cậy, có thứ tự |
| UDP `47801` | xe → PC | state stream 25 Hz | mất gói được, dùng gói mới nhất theo `seq` |
| UDP `47802` | PC → follower | relay state leader 25 Hz | mất gói được |

Không dùng WebSocket (TCP) cho stream tần số cao, vì khi WiFi mất gói, TCP retransmit có thể làm stream đứng 200 ms trở lên.

## 1. Discovery
Beacon (JSON, ≤ 200 B):
```json
{"t":"beacon","id":"a1b2c3","name":"robot-a1b2","ip":"192.168.1.42","fw":"0.1.0","role":"leader","bat":7.82,"rssi":-58}
```
PC cũng duyệt mDNS `_tworobot._tcp`. Nếu không thấy xe nào sau 5 s, PC quét `/api/info` trên /24 (64 request song song, timeout 300 ms).

## 2. HTTP API (xe)
| Method | Path | Mô tả |
|---|---|---|
| GET | `/api/info` | id, fw, role, ip, mac, bat, uptime |
| GET/PUT | `/api/config` | cấu hình đầy đủ ([parameters.md](parameters.md)). Apply vào RAM hoặc lưu flash, xem đầy đủ API ở [config-management §6](config-management.md#6-api-bổ-sung-protocolmd) |
| PUT | `/api/role` | `{"role":"leader"\|"follower"}` |
| PUT | `/api/path` | đường đi (xem §5). Xe trả về `{path_id, crc}` |
| POST | `/api/identify` | nháy LED và lắc bánh |
| POST | `/api/calib/{test}` | `motor_dir`, `deadband`, `straight`, `spin`, `range` |
| GET/PUT | `/api/wifi` | đổi SSID (khi ở AP mode) |

## 3. WebSocket messages
Khung chung: `{"t": <type>, "seq": <u32>, "ts": <ms clock xe hoặc PC>, ...}`. Mọi `cmd` đều được trả `ack` (cùng `seq`) hoặc `nack` kèm `err`.

**PC → xe**
```json
{"t":"hb"}                                            // 5 Hz; xe mất hb > link_timeout_ms thì BRAKE
{"t":"cmd","op":"arm","path_id":12}
{"t":"cmd","op":"prim","id":71,"prim":"DRIVE_TO","s_end":1200,"v_max":200}
{"t":"cmd","op":"prim","id":72,"prim":"TURN","deg":90}
{"t":"cmd","op":"prim","id":73,"prim":"FOLLOW","g_target":300,"s_limit":1200}
{"t":"cmd","op":"prim","id":74,"prim":"SNAPSHOT","n":15}
{"t":"cmd","op":"pause"} / {"t":"cmd","op":"resume"} / {"t":"cmd","op":"stop"}
{"t":"cmd","op":"estop"}                              // gửi lặp lại 3 lần
{"t":"cmd","op":"jog","v":100,"w":0}                  // chạy tay, hết hạn sau 300 ms
{"t":"sync","pc_ts":123456}                           // time sync
{"t":"cfg_set","key":"pid.wheel.kp","value":1.2}      // chỉnh nhanh, chỉ ghi RAM
```
**Xe → PC**
```json
{"t":"ack","seq":..} / {"t":"nack","seq":..,"err":"path_mismatch"}
{"t":"evt","e":"prim_done","id":71,"s":1198.5,"yaw":1.571}
{"t":"evt","e":"snapshot","id":74,"rho":352.1,"dpsi":-0.012,"std":1.8,"valid":true}
{"t":"evt","e":"fault","code":"LEADER_LOST"|"TOO_CLOSE"|"OBSTACLE"|"BAT_LOW"|"LINK_LOST"|"STALL"}
{"t":"sync_r","pc_ts":123456,"dev_ts":98765}
```

## 4. UDP state stream (xe → PC, 25 Hz, nhị phân little-endian)
```c
struct StatePkt {           // 48 B
  uint8_t  ver, role, phase, flags;  // flags: estop, fault, range_valid_l/r
  uint16_t seq, path_id;
  uint32_t t_ms;            // đồng hồ xe
  float    x, y, yaw;       // odometry (mm, rad), khung toạ độ đường
  float    s, v, w;         // tiến độ, vận tốc dài, vận tốc góc
  float    range_l, range_r;// mm, NAN nếu không hợp lệ
  float    gap_est;         // chỉ follower
  uint16_t bat_mv; int8_t rssi; uint8_t prim_id;
};
```
PC relay (UDP 47802) gửi cho follower: `{ver, seq, t_ms (đã đổi sang đồng hồ follower), s, v, phase}` của leader.

## 5. Time sync
PC gửi `sync` mỗi 1 s tới từng xe và nhận `sync_r`. Offset được ước lượng theo kiểu Cristian:
`off = dev_ts − (pc_send + rtt/2)`, giữ median của 8 mẫu gần nhất.
Khi relay, PC đổi `t_ms` từ đồng hồ leader sang đồng hồ follower: `t_F = t_L − off_L + off_F`.
Nhờ vậy follower bù được độ trễ: `s_L(now) ≈ s_L + v_L·(now − t_F)`.

## 6. Định dạng đường đi (PC → xe)
```json
{ "path_id": 12, "units": "mm",
  "start": {"x": 0, "y": 0, "yaw_deg": 0},
  "segments": [ {"len": 1200, "turn_after": 90},
                {"len": 600,  "turn_after": -90},
                {"len": 900,  "turn_after": 0} ],
  "corners_s": [1200, 1800],
  "follower": {"start_s": -300},
  "crc32": "8f3a12c0" }
```
`turn_after` ∈ {+90, −90, 0} (dương là rẽ trái). Hai xe nhận cùng nội dung. `s` được tính từ điểm start của leader.

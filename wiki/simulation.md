# Mô phỏng 2 robot trên PC và view dùng chung cho sim và xe thật

Nguồn: [đợt 3](../raw/2026-11-20-answers-3.md), [đợt 5](../raw/2026-11-20-answers-5.md).

> **Trạng thái cài đặt (2026-11-20):** giai đoạn đầu viết bằng **Python** (D20, D21), không phải C++ như §2 và §8 mô tả.
> Firmware ảo nằm ở `work/control-web/server/tworobot/fw/`, sim ở `.../sim/`. Robot ảo nối server qua `SimLink` trong process.
> Kiến trúc C++/CMake bên dưới vẫn là đích đến khi port firmware. Hướng dẫn chạy: [work/control-web/README.md](../work/control-web/README.md).

## 1. Mục tiêu
1. Phát triển và tinh chỉnh thuật toán **hoàn toàn trên máy tính** trước khi có xe.
2. Test toàn bộ hệ thống (server, discovery, coordinator, UI) với **robot ảo**.
3. Một **view trực quan duy nhất**. Sau này xe thật kết nối vào thì **hiển thị realtime trên đúng view đó**.
4. Có kịch bản hồi quy tự động, chấm điểm, chạy nhanh hơn thời gian thực.

## 2. Kiến trúc: robot ảo nói đúng giao thức của robot thật

```
            ┌────────────────── Web UI (1 view duy nhất) ───────────────────┐
            │ Canvas: lưới, đường, 2 xe, tia ToF, vết, vòng R_sw            │
            │ Panel: corner stepper, đồ thị, sự kiện, điều khiển sim, lỗi   │
            └───────────────────────────▲───────────────────────────────────┘
                                        │ WS /ui  — TelemetryFrame (schema chung)
            ┌───────────────────────────┴───────────────────────────────────┐
            │ Control Server (FastAPI): discovery, relay, coordinator,     │
            │ config, logger (JSONL), replay  —  KHÔNG phân biệt sim/thật  │
            └──────▲────────────────────▲──────────────────────▲────────────┘
     UDP/WS/HTTP   │       UDP/WS/HTTP  │                      │ đọc file
   ┌───────────────┴─┐   ┌──────────────┴───────────────┐  ┌───┴─────────┐
   │ ESP32 thật ×2   │   │ sim process (C++)            │  │ log JSONL   │
   │ app + hal_esp32 │   │ 2× (app + hal_sim) + World   │  │ (replay)    │
   └─────────────────┘   │ + kênh "truth" (sự thật)     │  └─────────────┘
                         └──────────────────────────────┘
```

- **Cùng một code điều khiển:** robot ảo chạy đúng `lib/core` (thuật toán), `lib/app` (primitive, máy trạng thái, safety) và `lib/proto` (giao thức) như ESP32. Chỉ khác lớp HAL.
- **Cùng giao thức:** mỗi robot ảo mở HTTP, WS, UDP trên `127.0.0.1` ở các port riêng và phát beacon. Server tìm thấy robot ảo như robot thật. Có thể thêm tay bằng `localhost:port`.
- **Kênh truth** (chỉ có ở sim): pose thật, tốc độ thật, va chạm. Server chuyển kênh này lên UI thành lớp overlay. Có lớp này mới thấy được **sai số ước lượng = ước lượng − thật**.
- **Chế độ nguồn trên UI:** `SIM`, `REAL`, `REPLAY`, và sau này `MIXED` (xe thật kết hợp xe ảo).

### 2.1 Tách HAL trong firmware
```cpp
struct IMotor   { virtual void setDuty(float d) = 0; virtual void brake() = 0; };
struct IEncoder { virtual int32_t ticks() = 0; virtual uint32_t lastEdgeUs() = 0; };
struct IImu     { virtual bool readGyroZ(float& rad_s) = 0; virtual bool readAccel(Vec3&) = 0; };
struct IRange   { virtual bool read(RangeSample&) = 0; };     // mm + status + t_us
struct IClock   { virtual uint32_t ms() = 0; virtual uint32_t us() = 0; };
struct IBattery { virtual float volts() = 0; };
struct IStorage { virtual bool load(Config&) = 0; virtual bool save(const Config&) = 0; };
struct ITransport { /* udp send/recv, ws send/recv, http routes */ };
```
`hal_esp32` dùng LEDC, PCNT, Wire, LittleFS và AsyncTCP. `hal_sim` đọc và ghi vào `World`.

## 3. Mô hình vật lý
Bước tích phân cố định **1 ms**. Các task của robot được lịch giả lập đúng tần số như trên xe (control 100 Hz, imu 200 Hz, ToF 30 Hz, net 25 Hz).
**Deterministic**: có seed RNG, cùng seed thì cùng kết quả.

| Khối | Mô hình |
|---|---|
| Nguồn | `V_bat(t)` giảm theo dung lượng, sụt theo dòng (nội trở). L298N: `V_motor = duty·(V_bat − V_drop(i))` |
| Motor DC (mỗi bánh) | `i = (V − k_e·ω_m)/R`, `τ = k_t·i·gear·η − τ_c·sign(ω) − b·ω`. Tham số **riêng từng bánh** (nhân hệ số lệch), mặc định lấy từ bài step response của xe thật |
| Bánh và sàn | Lực kéo giới hạn bởi `μ(x,y)·N`. Vượt giới hạn thì trượt (encoder đếm theo góc bánh, không theo quãng thật). **Vùng ma sát** vẽ được trên lưới |
| Thân xe | Rigid body 2D (khối lượng, quán tính), caster có lực cản và có thể kẹt ngắn |
| Encoder | Tick lượng tử từ góc bánh, timestamp cạnh xung (cho đo chu kỳ) |
| Gyro | `ω_đo = (1+e_s)·ω_thật + bias + random walk + nhiễu trắng`. Accel cho ZUPT |
| ToF200C | Chùm 25° được mô phỏng bằng **7 tia** chiếu vào polygon xe kia (thân và tấm phản xạ) và tường. Lấy khoảng nhỏ nhất. Nhiễu `σ(d)`, chu kỳ 33 ms, status lỗi khi góc tới quá xiên, ngoài tầm, hoặc tín hiệu yếu |
| Va chạm | Polygon–polygon và polygon–tường. Ghi lại thành metric, tuỳ chọn dừng sim |
| Mạng | Trễ (phân phối, jitter), mất gói % riêng từng kênh, đứt link theo lịch |

## 4. Chế độ chạy
| Chế độ | Dùng cho | Cách chạy |
|---|---|---|
| **Interactive** | Phát triển, demo, test UI | `sim serve --scenario s.yaml`. Realtime ×0.25–×4, pause, step |
| **Batch** | Hồi quy, quét tham số, CI | `sim run s.yaml --fast --seeds 50` cho ra `metrics.json` và log JSONL |
| **Replay** | Phân tích sau khi chạy (sim hoặc thật) | Mở log trên UI, kéo timeline |
| **Mixed** (M7) | Leader thật, follower ảo | ToF của xe ảo "nhìn" pose ước lượng của leader thật |

## 5. View: một giao diện cho cả 3 nguồn
- **Lớp hiển thị (bật/tắt từng lớp):** lưới và đường, pose ước lượng, pose thật (chỉ sim), vết đi, tia ToF (xanh khi hợp lệ, đỏ khi bị loại), vòng `R_sw`, vùng ma sát, điểm cua thực tế của leader và follower.
- **Panel:** corner stepper (10 bước), đồ thị cuộn (gap ước lượng/đo/mục tiêu, `v*`/`v_đo`/duty từng bánh, `Δψ`, hướng, `V_bat`), cờ `SAT/SLIP/STALL`, log sự kiện.
- **Điều khiển sim:** play, pause, step, tốc độ, reset, seed. Thanh trượt lệch motor, μ, nhiễu, trễ mạng.
  Nút gây lỗi: đẩy xe, che ToF, cắt link, caster kẹt, pin yếu.
- **Ghi và phát lại:** mọi phiên (sim hoặc thật) được ghi JSONL. Replay dùng **đúng view này**.
- Render ở 30–60 fps, nội suy giữa các frame 25 Hz.

### 5.1 TelemetryFrame (server → UI)
```json
{ "t": 12.340, "source": "SIM",
  "robots": [{ "id": "a1b2", "role": "leader",
     "est":   {"x":512.3,"y":0.4,"yaw":0.001,"s":512.3,"v":148.0,"w":0.0},
     "truth": {"x":515.0,"y":3.1,"yaw":0.006},
     "wheels":{"l":{"v_cmd":150,"v":147.2,"duty":0.61},"r":{"v_cmd":150,"v":148.9,"duty":0.58}},
     "tof":   {"l":{"mm":402,"st":0},"r":{"mm":399,"st":0}},
     "gap":   {"est":301.2,"meas":303.0,"target":300}, "dpsi": 0.004,
     "phase": "FOLLOW", "prim": 73, "flags": ["SAT"], "vbat": 11.9 }],
  "coord": {"corner": 2, "step": 6, "state": "RUNNING"},
  "events": [{"t":12.31,"e":"prim_done","robot":"a1b2","id":72}] }
```
Xe thật không có `truth`. Mọi trường khác giống nhau, nên UI không cần phân nhánh code.

## 6. Kịch bản (`work/sim/scenarios/*.yaml`)
```yaml
name: rect_2x1_motor_mismatch
map: {cell_mm: 100, cols: 30, rows: 20, friction_zones: [{rect: [10,0,5,3], mu: 0.5}]}
path: maps/rect_2x1.json
robots:
  leader:   {config: configs/robot_a.json, motor_scale: {l: 1.00, r: 0.85}}
  follower: {config: configs/robot_b.json, motor_scale: {l: 0.92, r: 1.05}}
network: {latency_ms: [10, 60], loss: 0.05}
faults:
  - {t: 8.0, type: tof_block, robot: follower, side: left, dur: 0.5}
  - {t: 15.0, type: link_drop, dur: 1.0}
pass: {gap_rmse_mm: 20, corner_err_mm: 30, collisions: 0}
```

## 7. Metric chấm điểm
Gap RMSE và gap sai lớn nhất (trên đoạn thẳng), gap nhỏ nhất (biên an toàn), sai số vị trí cua của follower so với chỗ leader thực sự quay, sai số hướng sau mỗi lần cua, sai số ước lượng so với truth, thời gian hoàn thành, số lần va chạm, fault, tỉ lệ thời gian bị `SAT`.

## 8. Công nghệ
- C++17 và **CMake** cho sim (dùng lại source `lib/*` của firmware). PlatformIO `native` vẫn dùng cho unit test.
- Thư viện header-only: **ArduinoJson** (dùng chung với firmware), **cpp-httplib** (HTTP), **IXWebSocket** (WS). Socket UDP qua POSIX.
- Server Python không cần biết sim, chỉ thêm một module `truth` để chuyển kênh truth lên UI.

# Thuật toán: đi theo đường và bám xe

Ràng buộc: cảm biến **2× ToF200C + MPU6050 + encoder**, giao tiếp **qua PC**, cua **90°** tại chỗ.
Ý tưởng chính: cả 2 xe biết cùng một đường, tham số hoá theo độ dài cung `s`.
- **Đoạn thẳng:** follower bám leader **tự chủ** bằng 2 cảm biến (đo gap và góc lệch tương đối), cộng encoder và đường biết trước.
- **Góc cua:** các bước rời rạc do PC điều phối. Ngay trước khi leader quay, follower **chụp số đo cảm biến** để biết chính xác còn bao xa tới điểm cua. Sau khi quay, follower dùng cảm biến để **căn song song lại với leader**.
  Gyro MPU6050 cho hướng tuyệt đối ngắn hạn (quay 90°, giữ thẳng). ToF cho **hướng tương đối với leader**, chính xác hơn trong dài hạn với follower.

## 1. Động học và điều khiển bánh

### 1.1 Động học vi sai
Track width `T`, bán kính bánh `r`:
```
v = (v_r + v_l)/2        ω = (v_r − v_l)/T
v_l = v − ω·T/2          v_r = v + ω·T/2
mm_per_tick = π·D / (PPR_motor · gear · 4)       (x4 quadrature)
```
Bù đường kính 2 bánh: `D_l = D·(1+ε)`, `D_r = D·(1−ε)`, với `ε` lấy từ bài hiệu chuẩn chạy thẳng.

### 1.2 Điều khiển bánh (L298N)
```
duty = FF_LUT(v_ref, bánh, chiều) / (V_bat − V_drop) + PI(v_ref − v_meas)
```
- Bảng tra FF lấy từ bài quét PWM, riêng cho từng bánh và từng chiều, bù theo điện áp pin.
- Vận tốc đo: đo chu kỳ xung khi chậm, đếm tick trên cửa sổ 50 ms khi nhanh.
- **Chi tiết xử lý motor mạnh yếu, ma sát khác nhau, bão hoà, thích nghi, tốc độ thấp: [motor-control.md](motor-control.md).**
- Anti-windup (clamp tích phân), giới hạn gia tốc trên `v_ref`.
- Khi dừng tại điểm: chuyển sang điều khiển **vị trí** cho từng bánh (P trên tick còn lại) ở tốc độ rất thấp, xong thì **phanh nhanh** L298N.

## 2. Odometry và hướng (encoder + gyro MPU6050)
- Quãng đường `Δs = (Δs_l + Δs_r)/2` lấy từ encoder (đáng tin, ít trượt khi đi thẳng).
- Hướng `ψ` dùng **complementary filter** hoặc KF 1 chiều:
  `ψ ← ψ + [ (1−β)·ω_gyro + β·ω_enc ]·dt`
  với `β` nhỏ (≈ 0.02–0.1) khi đi thẳng và `β ≈ 0` khi quay tại chỗ (bánh trượt, encoder sai, nên tin gyro hoàn toàn).
- **Bias gyro:** calib bắt buộc khi đứng yên, ZUPT mỗi lần dừng, bù theo nhiệt độ chip. Chi tiết ở §2.1. Scale gyro lấy từ bài quay 360° (§6).
- **Đi thẳng:** PI giữ hướng `ψ → ψ_seg`, kết hợp đồng bộ quãng đường 2 bánh. Gyro bắt được lệch nhanh, encoder giữ chuẩn dài hạn.
- **Quay 90° tại chỗ:** **vòng kín theo yaw gyro**. Profile `ω` hình thang tới `ψ_target = ψ_seg_mới`, lúc gần đích chuyển sang P-control với tốc độ tối thiểu vượt vùng chết. Dừng khi `|e_ψ| < turn_tol` và `|ω| ≈ 0`.
  Góc mục tiêu tính tuyệt đối (`ψ_seg_k = ψ_0 + Σ turns`) chứ không cộng dồn ±90 tương đối, nhờ đó sai số không tích luỹ.
- `T_eff` (track width hiệu dụng) vẫn được hiệu chuẩn, dùng cho feedforward và dự đoán khi gyro lỗi.
- Gyro MPU6050 trôi chậm (vài °/phút sau khi trừ bias), nên hướng tuyệt đối của leader vẫn trôi nhẹ trên đường dài. Follower bám tương đối nhờ §4.3 nên không bị ảnh hưởng.

### 2.1 Trôi MPU6050 và calib (nguồn: [đợt 6](../raw/2026-11-20-answers-6.md))

**Các thành phần sai số gyro Z** (đều có trong mô phỏng, `sim/world.py`):

| Thành phần | Mô hình sim (mặc định) | Xử lý |
|---|---|---|
| Bias lúc bật nguồn (mỗi lần mỗi khác) | ngẫu nhiên ±2.5 °/s | **calib khi đứng yên**, bắt buộc trước ARM |
| Trôi do nóng máy | bias đổi thêm ±0.3–0.8 °/s khi chip nóng 8 °C, τ = 90 s | ZUPT + **bù nhiệt độ** (học hệ số k) |
| Bias random walk | 0.01 °/s/√s | ZUPT ở mỗi lần dừng |
| Sai scale | 1 % | bài **quay 360°** → `imu.gyro_scale_z` |
| Nhiễu trắng + rung motor | 0.05 °/s + 0.002 °/s mỗi mm/s | tích phân cho trung bình, DLPF |
| Lượng tử | 1/131 °/s (±250 °/s) | không đáng kể |

**Thuật toán** (`fw/control.py`: `ImuCalibrator`, `HeadingEstimator`):
1. **Calib khi đứng yên:** lấy trung bình gyro trong `imu.calib_time_s` (2 s). Nếu encoder nhích, hoặc gyro dao động quá `imu.calib_max_spread_dps` (bị chạm, rung) thì làm lại, tối đa 5 lần rồi báo lỗi `IMU_NOT_STILL`.
   - Tự chạy khi bật nguồn (`imu.auto_calib`), và chạy lại ngay trước mỗi lần chạy: coordinator có **bước 0 "Calib IMU"** cho cả 2 xe (`imu.recal_before_run`).
   - Chưa calib thì: ARM bị từ chối (`imu_uncal`), và **không tích phân hướng khi đứng yên**, vì bias chưa biết.
2. **ZUPT:** khi bánh phanh, encoder không nhích, và |ω| < `imu.still_gyro_dps` trong hơn 0.3 s, bias được lọc theo số đo (`imu.zupt_gain`) và hướng không bị tích phân. Xe dừng ở mỗi góc cua, nên bias được làm mới khoảng 10–20 s một lần.
3. **Bù nhiệt:** mỗi lần đứng yên đủ lâu (và cứ 5 s khi đứng lâu), ghi một điểm (T_chip, bias). Hồi quy tuyến tính trên các điểm này cho ra hệ số k (°/s/°C). Khi đang chạy, bias được tính lại: `bias = bias_neo + k·(T − T_neo)`.
   - k là hằng số riêng của từng chip. Robot tự học k trong lúc nằm chờ sau khi bật nguồn. Nút "Lưu k nhiệt" lưu k vào `imu.temp_coeff_dps_c` trên flash, nên từ lần bật sau đã bù đúng ngay từ đoạn đầu.
4. **Không học bias bằng encoder khi đang chạy:** đường kính 2 bánh lệch 1 % đã làm `ω_enc` sai khoảng 0.7 °/s ở 150 mm/s, lớn hơn chính bias cần bám.

**Kết quả mô phỏng** (rect/zigzag, 3 seed, chờ 60 s sau bật nguồn, mô hình gyro đầy đủ):

| Cấu hình | Sai hướng max (tb) | Leader lệch điểm cuối max (tb) |
|---|---|---|
| Không calib, không ZUPT | 36° | 480 mm |
| Chỉ calib 1 lần lúc bật nguồn | 26° | 480 mm |
| Calib + ZUPT + bù nhiệt, **chưa** hiệu chuẩn scale | 4.3° (2.6°) | 66 mm (40) |
| Như trên + **đã** hiệu chuẩn scale (quay 360°) | 1.7° (1.2°) | 33 mm (21) |
| Riêng trôi nóng máy: không bù / bù với k đã lưu | 2.6° / 0.8–1.2° | 55 / 8 mm |

Kết luận: **calib lúc đứng yên + ZUPT là bắt buộc. Bù nhiệt và hiệu chuẩn scale là 2 bước kế tiếp quan trọng nhất.** Mọi cấu hình đều chạy hết đường không va chạm, vì follower bám bằng ToF. Sai số hướng chủ yếu làm leader đi lệch so với bản vẽ.

**Khuyến nghị khi dùng xe thật:** bật nguồn 1–2 phút trước khi chạy, để chip ấm lên và robot tự học k. Đặt xe nằm yên khi bật nguồn và khi bấm Start. Làm bài quay 360° một lần cho mỗi xe.

## 3. Leader: bám đường
- Mỗi đoạn thẳng là một primitive `DRIVE_TO(s_end)` với profile hình thang (`a_max`, `v_cruise`). Tốc độ giảm về `v_corner` trong `approach_dist` cuối, rồi dừng bằng điều khiển vị trí.
- Leader giữ hướng `ψ_seg` bằng gyro và encoder (§2). Leader không sửa được lệch ngang tuyệt đối vì không có mốc ngoài. Lệch ngang chỉ đến từ trôi hướng nên nhỏ.
- An toàn: cảm biến phía trước của leader thấy vật < `obstacle_stop` thì phanh và gửi `fault OBSTACLE`.
- An toàn 2 xe: trong `FOLLOW`, một hoặc hai mắt ToF của follower giảm đột ngột ≥ `safety.tof_anomaly_jump_mm` hoặc mất tín hiệu trong `safety.tof_anomaly_samples` mẫu liên tiếp → follower phanh, báo `OBSTACLE_INTRUSION`; PC phanh leader và Web UI hiện cảnh báo.

## 4. Follower: bám trên đoạn thẳng (tự chủ, `FOLLOW`)

### 4.1 Mô hình đo
Cảm biến đặt ở `(x_s, ±b/2)` so với tâm trục follower, chùm song song hướng về trước.
Mặt sau leader (tấm phản xạ) cách tâm trục leader `x_rear` về phía sau:
```
d̄   = (d_l + d_r)/2
Δψ  = atan2(d_l − d_r, b)                 // góc lệch tương đối giữa 2 xe (dấu: d_l > d_r ⇒ leader xoay sang trái)
ρ   = x_s + d̄·cos(Δψ) + x_rear            // gap tâm–tâm
```
**Giới hạn:** chiếu lên mặt phẳng, 2 chùm song song không đo được lệch ngang. Lệch ngang chỉ phát hiện được khi một chùm trượt ra ngoài mép tấm (số đo nhảy xa).
Vùng ngang hợp lệ là `±(W_plate − b)/2`. Ví dụ tấm 120 mm, `b` = 60 mm thì ±30 mm.

### 4.2 Bộ lọc Kalman cho gap
Trạng thái `x = [g, v_L]ᵀ`, đầu vào là `v_F` (encoder).
```
Dự đoán (100 Hz):   g ← g + (v_L − v_F)·dt ;   v_L ← v_L      (Q từ q_a: nhiễu gia tốc leader)
Đo ToF (20–50 Hz):  z = ρ                        R = σ²_range (theo loại cảm biến, tăng khi |Δψ| lớn)
Đo relay (25 Hz, tuỳ chọn): z = v_L_relay        R lớn. Đã bù trễ bằng timestamp (protocol §5)
```
- Chỉ dùng cảm biến vẫn quan sát được `v_L` (qua đạo hàm của `g`). Relay chỉ giúp hội tụ nhanh và chạy mượt hơn. **Mất relay thì bám vẫn hoạt động.**
- **Gating:** bỏ số đo khi `ν²/S > χ²(1, 0.99)` (≈ 6.6), khi `|d_l − d_r| > dmax_diff`, hoặc khi ngoài `[min_valid, max_valid]`. Dùng 1 cảm biến nếu cảm biến kia bị loại. Bỏ cả 2 quá `lost_timeout_ms` thì `BRAKE` và báo `LEADER_LOST`.
- Tiền xử lý: median 3 mẫu mỗi cảm biến. Bỏ mẫu có `RangeStatus` khác 0 (VL53L0X báo lỗi sigma, signal, phase). Thêm kiểm tra tốc độ thay đổi.

### 4.3 Luật điều khiển follower
```
v_cmd = clamp( v̂_L + kp_g·(ĝ − g*) + ki_g·∫(ĝ − g*) , 0 , v_max(s_F) )
ω_cmd = −k_psi · Δψ_f  −  k_edge · edge_term
```
- `v̂_L` là feedforward, tránh trễ pha. `v_max(s_F)` theo profile của đường, nên follower cũng tự chậm lại gần góc cua.
- `Δψ_f`: `Δψ` qua complementary filter với `ω` gyro của follower (dự đoán bằng `−ω_F·dt` giữa các mẫu ToF ở 30 Hz). Luật này đưa follower **song song với leader** và đi theo hướng thực của leader, không theo hướng odometry của chính nó.
  Nếu ToF mất tạm thời, follower giữ hướng `ψ_seg` bằng gyro (giống leader).
- `edge_term`: +1 hoặc −1 khi một chùm mất mép tấm. Follower lái nhẹ về phía chùm vẫn còn thấy leader.
- Cứng: `ĝ < g_min_safe` thì `v = 0`.
- Cuối cùng: `(v_cmd, ω_cmd)` qua §1.1 thành `(v_l, v_r)` rồi qua §1.2 thành PWM.
  Đây chính là phần "**ước lượng vận tốc từng bánh**" từ đường biết trước, bộ lọc gap và cảm biến.

`s_F` vẫn được tích phân từ encoder để biết đang ở đoạn nào và khi nào vào vùng góc cua.

## 5. Quy trình qua góc cua (PC điều phối)

**Hình học an toàn** (thân hộp quay quanh tâm trục bánh):
- `R_sw = sqrt(max(axle_to_front, axle_to_rear)² + (width/2)²)`: bán kính quét khi quay tại chỗ.
- Khi leader quay tại `C`: mũi follower phải ngoài `R_sw + margin`, tức là
  `g_hold ≥ R_sw + axle_to_front_F + margin`.
- Khi follower quay tại `C`: đuôi leader phải ngoài `R_sw + margin`, tức là
  `exit_dist ≥ R_sw + axle_to_rear_L + margin`.
- `g_hold − x_s − x_rear` phải nằm trong vùng đo hợp lệ của cảm biến.

**Trước đoạn đầu tiên:**
1. Bước 0a: calib IMU cho cả 2 xe (đứng yên, §2.1).
2. Bước 0b: **kiểm tra trước khi chạy** (§5.1).
3. ARM. Nếu xe sau lệch góc nhỏ thì tự ALIGN.

### 5.1 Đặt xe lúc đầu không chuẩn và bước kiểm tra trước khi chạy

**Ảnh hưởng khi đặt lệch** (sim, rect + zigzag, 6 lần mỗi dòng, chưa bật kiểm tra):

| Đặt lệch | Chạy xong | Ảnh hưởng | Tự xử lý được? |
|---|---|---|---|
| Xe sau lệch góc 3–8° | 6/6 | không đáng kể | **có**: Δψ từ ToF, ALIGN |
| Xe sau gần hơn 60 mm / xa hơn 150 mm | 6/6 | xa thì gap dao động lúc đầu (RMSE 55 mm) | **có**: FOLLOW đưa gap về mục tiêu |
| Xe sau lệch **ngang** 10 / 25 / 40 mm | 6/6 | lệch điểm cua 27 / 36 / 47 mm | **không**: lệch ngang không quan sát được |
| Xe trước lệch **góc** 2° / 5° | 6/6 | leader lệch điểm cuối 94 / 223 mm | **không**: không có mốc tuyệt đối |

**Bước kiểm tra** (`coordinator.preflight`; nút "✓ Kiểm tra" chạy thử không cần Start; không đạt thì không ARM và báo cần chỉnh gì):
- Mỗi xe: đang kết nối, không FAULT/ESTOP, pin đủ, IMU đã calib.
- Xe sau SNAPSHOT: **2 mắt đều thấy** đuôi xe trước. Lỗi phân loại: không thấy gì / chỉ 1 mắt (lệch ngang) / 2 mắt chênh nhiều.
- Gap ∈ `g_target ± start.gap_tol_mm` (40). Báo cần đẩy lên hay lùi ra bao nhiêu mm.
- |Δψ| ≤ `start.max_dpsi_deg` (6°). Nếu > `start.align_dpsi_deg` (1.5°) thì tự ALIGN sau khi ARM.
- Phía trước xe đầu trống ≥ `start.front_clear_mm` (250).

**Giới hạn (đã kiểm chứng bằng sim):** chùm ToF 25° ở cự ly khoảng 15 cm toả ra khoảng ±35 mm, nên **lệch ngang dưới khoảng 70 mm KHÔNG phát hiện được**. Lệch 30–60 mm vẫn qua kiểm tra nhưng làm lệch điểm cua 31–39 mm. Góc lệch tuyệt đối của xe trước cũng không đo được.
→ **Phải đặt xe bằng dụng cụ:** vạch kẻ sàn hoặc thước chữ L căn mép 2 xe thẳng hàng, xe trước căn theo vạch hướng đi đầu tiên.

**Các bước cho góc `k`** (coordinator trên PC; mỗi bước là lệnh primitive và chờ `prim_done`):

| # | Leader | Follower | Điều kiện sang bước sau |
|---|---|---|---|
| 1 | `DRIVE_TO(s_k)`, dừng tại `C_k` | `FOLLOW(g*→g_hold khi gần góc)`, giữ ở `g_hold` | cả 2 đứng yên |
| 2 | `HOLD` | **`SNAPSHOT`**: trung bình N mẫu, cho ra `ρ_k` và `Δψ_k` | `std < snap_std_max` |
| 3 | `HOLD` | `ALIGN` nếu `|Δψ_k| > align_tol` (song song lại trước khi đi) | xong |
| 4 | `TURN(±90)` | `HOLD` | xong |
| 5 | `DRIVE_TO(s_k + exit_dist)` | `HOLD` | xong |
| 6 | `HOLD` | `DRIVE_TO(s_F + ρ_k)`, **chỉ dùng encoder** (cảm biến lúc này nhìn ra khoảng trống) | xong |
| 7 | `HOLD` | `TURN(±90)` | xong |
| 8 | `HOLD` | **`REACQUIRE(ρ_exp = exit_dist)`**: kiểm tra cả 2 chùm thấy leader, `|ρ − ρ_exp| < tol`. Nếu không thấy thì quét ±10° tìm hướng có `d̄` nhỏ nhất | thấy leader |
| 9 | `HOLD` | `ALIGN` (quay tới `Δψ≈0`). Cập nhật lại `s_F = s_k`, reset KF với `g = ρ` | xong |
| 10 | `DRIVE_TO(s_{k+1})` (đoạn tiếp) | `FOLLOW(g*)` | quay về bước 1 cho góc `k+1` |

Ghi chú:
- Bước 2 là **"neo" quan trọng nhất**. Lúc này cả 2 đứng yên và thẳng hàng, nên `ρ_k` đo chính xác khoảng cách từ follower tới điểm leader đã thực sự đứng.
  Follower sẽ quay **đúng chỗ leader quay** dù odometry 2 xe đã lệch nhau.
- Bước 4 và 7 quay vòng kín theo gyro (sai ~1°). Bước 8 và 9 tinh chỉnh tiếp bằng ToF để follower song song đúng với leader thực tế.
- Bước 1–2 đứng yên nên tận dụng để **cập nhật bias gyro** cho cả 2 xe.
- Nếu `seg_len(k+1) < exit_dist`, validator từ chối đường này (hoặc tăng ô lưới).
- Bất kỳ bước nào quá `step_timeout` hoặc có `fault` thì PC ra lệnh `BRAKE` cả 2, chuyển sang `PAUSED_FAULT` và hiện trên UI. Người dùng có thể Retry bước đó, jog tay rồi Retry, hoặc Abort.
- Quy trình chậm nhưng chắc. Tối ưu sau: gộp bước 5 với bước 6–7 chạy song song khi khoảng cách đủ an toàn.

## 6. Hiệu chuẩn
| Bài | Cách làm | Kết quả |
|---|---|---|
| Chiều motor/encoder | quay từng bánh tiến 1 s | `invert_*` |
| Vùng chết | tăng PWM từng bước tới khi bánh quay, mỗi bánh mỗi chiều | `ks`, `kv` |
| Thẳng 1 m | **wizard**: xe chạy `CAL_DRIVE` 1000 mm (giữ hướng gyro), người dùng đo quãng thật → `D_mới = D_cũ × thật / encoder` | `wheel.diameter_mm` |
| Gyro bias | đứng yên 5 s | `gyro_bias_z`, `σ` nhiễu |
| Quay | **wizard**: `CAL_TURN` 360° (góc cộng dồn), người dùng đo góc thật → `scale_mới = cũ × thật / gyro`; `T_eff = (cung phải − cung trái) / góc thật` | `imu.gyro_scale_z`, `wheel.track_eff_mm` |
| ToF lệch 2 mắt | **quét ±40° trước mặt phẳng** (§6.1), nút "Calib ToF" | `range.offset_l/r_mm` |
| ToF khoảng cách (đặt sẵn) | đặt xe sau cách đuôi xe trước đúng X mm (vd. 100), nhập X → dời đều 2 offset (giữ chênh lệch) hoặc đặt riêng từng mắt | `range.offset_l/r_mm` |
| ToF tuyệt đối (tuỳ chọn) | như trên + nhập khoảng cách tường `wall_mm` | offset tuyệt đối |
| Bố trí | đặt follower thẳng sau leader ở gap đã biết | xác nhận `x_s`, `x_rear`, `Δψ` offset lắp |

### 6.1 Hai mắt ToF lắp không thẳng hàng (nguồn: câu hỏi sau đợt 7)

**Vấn đề:** `Δψ = atan2(d_r − d_l, b)` với `b` chỉ 60 mm. Một hằng số lệch `c` trong `d_r − d_l` sẽ làm Δψ lệch `atan(c/b)`:
mắt nhô trước/sau 2 mm, hoặc 2 module VL53L0X có offset khác nhau vài mm (rất phổ biến), là đã lệch 2–5°.
Follower khi đó "song song" với leader theo một góc sai, đi chéo dần, mất mép tấm phản xạ, và ALIGN ở góc cua căn sai.

| Loại sai lệch lắp | Ảnh hưởng | Xử lý |
|---|---|---|
| Mắt nhô trước/sau khác nhau (Δx), offset riêng từng module | hằng số `c` trong `d_r − d_l` → **Δψ lệch, nặng nhất** | hiệu chuẩn quét tường → bù `range.offset_l/r_mm` |
| Khoảng cách 2 mắt khác thiết kế | sai **tỉ lệ** Δψ (không lệch điểm 0) | bài hiệu chuẩn báo `b` hiệu dụng để kiểm tra; ít quan trọng |
| Mắt quay lệch trái/phải vài độ | nhỏ (chùm rộng 25°), đổi vùng tấm bị chiếu | gá chắc, kiểm tra bằng mắt |
| Mắt chúi xuống / nghiêng lên | chùm chạm sàn → số đo ngắn sai (sim 2D chưa mô phỏng) | đặt mắt cao ≥ 40 mm, hơi ngẩng; tấm phản xạ đủ cao |

**Bài hiệu chuẩn `TOF_CAL`** (`fw/primitives.py`; đã calib gyro; chạy được khi xe IDLE; khoảng 25 s):
1. Đặt xe trước một mặt phẳng **rộng** (tường/hộp ≥ 40 cm), cách 15–30 cm, gần vuông góc (±10°).
2. Xe quay ±40° đi rồi về, ghi (góc gyro, d_l, d_r).
3. Chùm 25° làm d̄ **phẳng** khi gần vuông góc và tăng vọt khi xoay quá nửa góc chùm. Đồ thị đối xứng quanh hướng vuông góc, nên **θ0 = trung điểm 2 mép đoạn phẳng**. Không dùng "điểm d̄ nhỏ nhất", vì đoạn đó phẳng nên điểm nhỏ nhất không rõ.
4. Trong đoạn phẳng fit tuyến tính `d_r − d_l = c − b·(θ − θ0)`. Bù `off_r − off_l = −c` (chia đều 2 bên). Nếu nhập khoảng cách tường thì bù luôn offset tuyệt đối.

**Kết quả mô phỏng:**
- Độ chính xác: 30/30 lần thành công (xe đặt lệch ±9°, cách 150/250 mm). Sai `c` sau hiệu chuẩn: trung bình 0.66 mm, tối đa 1.5 mm, tức Δψ còn lệch trung bình 0.6°, tối đa 1.5°.
- Ảnh hưởng lên cả lần chạy (rect + zigzag, 6 lần mỗi dòng):

| Lắp đặt | Chạy xong | Va chạm | Lệch điểm cua max |
|---|---|---|---|
| Chuẩn | 6/6 | 0 | 14 mm |
| Lệch vừa (c = −4 mm, Δψ lệch 3.8°), **chưa** calib | 4/6 | 0 | 62 mm |
| Lệch vừa, **đã** calib | 6/6 | 0 | 34 mm |
| Lệch nặng (c = 13.5 mm, Δψ lệch 12.7°), **chưa** calib | **0/6** (mất leader) | 0 | — |
| Lệch nặng, **đã** calib | 6/6 | 0 | 33 mm |
| Lệch nặng, đã calib, **baseline 90 mm** | 6/6 | 0 | 30 mm |

**Khuyến nghị lắp đặt:**
- Gá 2 module trên **cùng một thanh cứng** (in 3D/mica), mặt kính cùng một mặt phẳng, cùng độ cao, chùm song song.
- **Baseline càng rộng càng tốt** (80–100 mm), vì sai số Δψ tỉ lệ nghịch với b. Khi đó tấm phản xạ đuôi cũng phải rộng hơn: ≥ b + 2 × dung sai ngang (vd. b = 90 cần tấm ≥ 150 mm).
- Luôn chạy "Calib ToF" sau khi lắp hoặc tháo lắp cảm biến, rồi Save flash.

## 7. Kiểm chứng
1. Unit test lõi trên `native`: kinematics, profile, dừng tại điểm, KF, gating, máy trạng thái primitive.
2. Sim 2 xe ([simulation.md](simulation.md)): motor DC lệch từng bánh, vùng ma sát, trượt bánh khi quay (sai ngẫu nhiên 1–3°), nhiễu encoder, gyro có bias/random walk, mô hình ToF (nhiễu ~3 %, FOV 25°, mép tấm), trễ và mất gói relay. Coordinator dùng chính code server qua "robot ảo".
3. Kịch bản: thẳng 3 m, chữ L, chữ nhật, zig-zag ô nhỏ, góc liên tiếp ở `seg_min`, mất relay, che cảm biến.
4. Phần cứng: bánh, rồi 1 xe (thẳng, quay), rồi follower bám trên đoạn thẳng, rồi 1 góc cua, rồi cả đường.

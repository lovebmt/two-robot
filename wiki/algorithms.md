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
duty = sign(v_ref)·(ks + kv·|v_ref|) + PI(v_ref − v_meas)       // feedforward + PI
```
- `ks` (vùng chết) và `kv` lấy từ bài quét PWM, đo riêng cho từng bánh và từng chiều.
- Vận tốc đo được: đạo hàm tick, lọc low-pass (α ≈ 0.3) hoặc đo chu kỳ xung khi chạy chậm.
- Anti-windup (clamp tích phân), giới hạn gia tốc trên `v_ref`.
- Khi dừng tại điểm: chuyển sang điều khiển **vị trí** cho từng bánh (P trên tick còn lại) ở tốc độ rất thấp, xong thì **phanh nhanh** L298N.

## 2. Odometry và hướng (encoder + gyro MPU6050)
- Quãng đường `Δs = (Δs_l + Δs_r)/2` lấy từ encoder (đáng tin, ít trượt khi đi thẳng).
- Hướng `ψ` dùng **complementary filter** hoặc KF 1 chiều:
  `ψ ← ψ + [ (1−β)·ω_gyro + β·ω_enc ]·dt`
  với `β` nhỏ (≈ 0.02–0.1) khi đi thẳng và `β ≈ 0` khi quay tại chỗ (bánh trượt, encoder sai, nên tin gyro hoàn toàn).
- **Bias gyro:** ước lượng lúc đứng yên (encoder = 0 và |accel| ổn định). Bias được cập nhật lại ở mỗi lần dừng tại góc cua (ZUPT). Thêm hệ số scale gyro từ bài quay 360°.
- **Đi thẳng:** PI giữ hướng `ψ → ψ_seg`, kết hợp đồng bộ quãng đường 2 bánh. Gyro bắt được lệch nhanh, encoder giữ chuẩn dài hạn.
- **Quay 90° tại chỗ:** **vòng kín theo yaw gyro**. Profile `ω` hình thang tới `ψ_target = ψ_seg_mới`, lúc gần đích chuyển sang P-control với tốc độ tối thiểu vượt vùng chết. Dừng khi `|e_ψ| < turn_tol` và `|ω| ≈ 0`.
  Góc mục tiêu tính tuyệt đối (`ψ_seg_k = ψ_0 + Σ turns`) chứ không cộng dồn ±90 tương đối, nhờ đó sai số không tích luỹ.
- `T_eff` (track width hiệu dụng) vẫn được hiệu chuẩn, dùng cho feedforward và dự đoán khi gyro lỗi.
- Gyro MPU6050 trôi chậm (vài °/phút sau khi trừ bias), nên hướng tuyệt đối của leader vẫn trôi nhẹ trên đường dài. Follower bám tương đối nhờ §4.3 nên không bị ảnh hưởng.

## 3. Leader: bám đường
- Mỗi đoạn thẳng là một primitive `DRIVE_TO(s_end)` với profile hình thang (`a_max`, `v_cruise`). Tốc độ giảm về `v_corner` trong `approach_dist` cuối, rồi dừng bằng điều khiển vị trí.
- Leader giữ hướng `ψ_seg` bằng gyro và encoder (§2). Leader không sửa được lệch ngang tuyệt đối vì không có mốc ngoài. Lệch ngang chỉ đến từ trôi hướng nên nhỏ.
- An toàn: cảm biến phía trước của leader thấy vật < `obstacle_stop` thì phanh và gửi `fault OBSTACLE`.

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
| Thẳng 1 m | chạy 1000 mm theo encoder, người dùng nhập quãng đo thực và độ lệch ngang | `D` (scale), `ε` |
| Gyro bias | đứng yên 5 s | `gyro_bias_z`, `σ` nhiễu |
| Quay | 4×90° và 360° theo gyro, nhập góc thực (căn theo vạch sàn) | `gyro_scale`, `T_eff` |
| ToF | tấm phẳng ở 100/200/300/500 mm | `offset`, `scale` từng cảm biến, `σ` |
| Bố trí | đặt follower thẳng sau leader ở gap đã biết | xác nhận `x_s`, `x_rear`, `Δψ` offset lắp |

## 7. Kiểm chứng
1. Unit test lõi trên `native`: kinematics, profile, dừng tại điểm, KF, gating, máy trạng thái primitive.
2. Sim 2 xe: motor bậc 1 có vùng chết, trượt bánh khi quay (sai ngẫu nhiên 1–3°), nhiễu encoder, gyro có bias/random walk, mô hình ToF (nhiễu ~3 %, FOV 25°, mép tấm), trễ và mất gói relay. Coordinator dùng chính code server qua "robot ảo".
3. Kịch bản: thẳng 3 m, chữ L, chữ nhật, zig-zag ô nhỏ, góc liên tiếp ở `seg_min`, mất relay, che cảm biến.
4. Phần cứng: bánh, rồi 1 xe (thẳng, quay), rồi follower bám trên đoạn thẳng, rồi 1 góc cua, rồi cả đường.

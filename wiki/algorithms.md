# Thuật toán: đi theo đường và bám xe

Đây là phần khó nhất của dự án. Ý tưởng chính: **cả 2 xe biết cùng một đường và cùng tham số hoá đường theo độ dài cung `s`**.
Bài toán bám xe được đưa về điều khiển **khoảng cách dọc theo đường** `g = s_L − s_F`. Hướng ngang do bộ bám đường lo.
Cảm biến khoảng cách và dữ liệu radio dùng để **hiệu chỉnh** `g` và góc lệch. Không lái trực tiếp theo cảm biến.

## 1. Động học vi sai

Track width `T`, bán kính bánh `r`, tốc độ góc bánh `φ̇_l`, `φ̇_r`:

```
v_l = r·φ̇_l    v_r = r·φ̇_r
v   = (v_r + v_l)/2          ω = (v_r − v_l)/T
nghịch:  v_l = v − ω·T/2      v_r = v + ω·T/2
```
Encoder: `mm_per_tick = π·D / (CPR·gear·4)` (x4 decoding).

## 2. Ước lượng pose từng xe (odometry + gyro)

- Odometry 100 Hz tích phân `(v, ω)` từ encoder.
- Góc quay: **complementary/EKF nhỏ** gộp `ω_enc` và `ω_gyro`. Gyro được ưu tiên khi quay tại chỗ (bánh dễ trượt), encoder được ưu tiên khi đi thẳng. Bias gyro được hiệu chỉnh lúc xe đứng yên (ZUPT).
- Chiếu pose lên đường để lấy `s` (tiến độ), `e_y` (sai số ngang), `e_ψ` (sai số góc).

## 3. Bám đường (cả 2 xe)

- Đường v1 gồm các đoạn thẳng và các lần quay tại chỗ, nên bộ bám đường trên đoạn thẳng là **Stanley** hoặc **pure pursuit** với lookahead nhỏ:
  `ω = −k_ψ·e_ψ − atan(k_y·e_y / (v+ε))`
- **Profile vận tốc** `v_max(s)`: hình thang hoặc S-curve với `a_max` và `j_max`, giảm về `v_corner` trong vùng `approach_dist` trước góc cua và dừng đúng tại `s_k`.
- Bánh xe: mỗi bánh một **PI + feedforward** (`PWM = kv·v_ref + ks·sign(v_ref) + PI(e)`), có bù vùng chết động cơ và anti-windup.

## 4. Bộ lọc bám xe (follower)

### 4.1 Mô hình đo của 2 cảm biến
Cảm biến đặt ở `(x_s, ±b/2)` so với tâm trục bánh follower, chùm hướng thẳng về trước (hoặc nghiêng ra ngoài góc `α`).
Mặt sau leader là mặt phẳng cách tâm trục leader một đoạn `x_rear` về phía sau. Với 2 số đo `d_l`, `d_r`:

```
d̄  = (d_l + d_r)/2                       khoảng cách mặt sau
Δψ = atan2(d_l − d_r, b)                 góc lệch tương đối giữa 2 xe
khoảng cách tâm–tâm:  ρ ≈ x_s + d̄·cos(Δψ) + x_rear
```

**Giới hạn cần biết:** 2 chùm song song chiếu lên mặt phẳng chỉ đo được *khoảng cách* và *góc tương đối*, **không đo trực tiếp được lệch ngang**.
Lệch ngang chỉ quan sát gián tiếp khi một chùm trượt khỏi mép tấm phản xạ (số đo nhảy lớn). Vì vậy:
- lệch ngang chủ yếu dựa vào bám đường và odometry,
- `Δψ` dùng để hiệu chỉnh góc follower, vì góc leader đã biết từ đường hoặc từ ESP-NOW: `ψ_F ≈ ψ_L − Δψ`,
- tuỳ chọn: nghiêng 2 cảm biến ra ngoài ±10–15° và dùng tấm phản xạ có mép để có thêm thông tin ngang (xem [open-questions Q6](open-questions.md)).

### 4.2 Kalman filter cho gap
Trạng thái `x = [g, v_L]ᵀ` (gap dọc đường tâm–tâm và vận tốc leader).

```
Dự đoán (100 Hz):   g⁺ = g + (v_L − v_F)·dt      v_L⁺ = v_L (+ nhiễu gia tốc q_a)
Đo 1 – ToF (30–50 Hz, chỉ trên đoạn thẳng):      z₁ = ρ             R₁ nhỏ
Đo 2 – radio (50 Hz):  z₂ = s_L(t−τ) + v_L·τ − s_F   R₂ lớn hơn (drift odometry)
Đo 3 – radio:          z₃ = v_L                   R₃
```
- **Gating**: Mahalanobis `ν²/S > χ²` thì bỏ số đo ToF. Trường hợp này xảy ra khi leader đang quay, chùm trượt mép, hoặc `|d_l − d_r|` quá lớn.
- **Khi leader quay hoặc ở gần góc cua**, mặt sau leader xoay nên ToF không còn tin được. Lúc đó tắt cập nhật ToF và chạy bằng `z₂`, `z₃` cùng odometry.
- Mất cả radio lẫn ToF quá `lost_timeout_ms` thì dừng.
- Lợi ích phụ: chênh lệch dài hạn giữa `z₁` và `z₂` cho biết sai số odometry tương đối, có thể dùng để chỉnh `s_F`.

### 4.3 Luật điều khiển vận tốc follower
```
v_F_cmd = clamp( v̂_L + k_p·(ĝ − g*) + k_i·∫(ĝ − g*) ,  0 ,  v_max(s_F) )
```
- `v̂_L` là feedforward, giúp không trễ pha.
- `v_max(s_F)` dùng cùng profile của đường, nên follower cũng tự chậm lại trước góc cua.
- Ràng buộc cứng: `ĝ < g_min_safe` thì `v_F = 0`.
- `ω_F` lấy từ bộ bám đường (§3), sau đó tính `v_l`, `v_r` (§1) và đưa vào PI bánh.

**Kết quả:** vận tốc từng bánh follower được ước lượng từ đường biết trước (hướng), bộ lọc gap (tốc độ dài) và cảm biến (hiệu chỉnh).

## 5. Giao thức qua góc cua (stop-and-go)

Ký hiệu: góc cua thứ `k` ở `s_k`. `R_sw` là bán kính quét khi quay tại chỗ (tâm trục tới góc xa nhất của thân xe).
Hai xe đều hình hộp và quay quanh trục bánh, nên khi một xe quay tại `C_k`, xe kia phải cách tâm `C_k` **≥ R_sw + R_body_reach + margin**.
Từ đó suy ra gap tối thiểu ở góc cua `g_turn_min`. Web kiểm tra điều này ở F9.

### Máy trạng thái leader (tại góc k)
```
DRIVE → APPROACH (giảm về v_corner) → STOP_AT(s_k) → TURN (vòng kín theo gyro, ±1°)
      → EXIT (tiến chậm thêm exit_dist ≥ clear_dist) → [chờ follower nếu g > g_max] → DRIVE
```
### Máy trạng thái follower (tại góc k)
```
FOLLOW → APPROACH_k : chậm lại và giữ cách C_k ≥ hold_dist cho tới khi nhận LEADER_EXIT_DONE(k)
       → ADVANCE_k  : tiến tới đúng s_k (dừng theo odometry/encoder, không dùng ToF)
       → TURN_k     : quay tại chỗ, vòng kín theo gyro
       → REACQUIRE  : mở lại cập nhật ToF, chờ số đo hợp lệ ổn định N mẫu
       → FOLLOW
```
Sự kiện qua ESP-NOW (có ACK, gửi lặp lại tới khi nhận ACK):
`L_AT_CORNER(k)`, `L_TURN_DONE(k)`, `L_EXIT_DONE(k)`, `F_AT_CORNER(k)`, `F_TURN_DONE(k)`.

Luồng ví dụ (đúng như brief): leader dừng ở C và quay, follower đứng chờ trước C. Leader tiến tới một chút, follower tiến tới C rồi quay. Sau đó cả 2 tăng tốc trở lại.
Leader chỉ được vào góc cua `k+1` khi follower đã `F_TURN_DONE(k)`. Nhờ vậy các góc gần nhau vẫn an toàn.

## 6. Hiệu chuẩn
- **Thẳng 1 m** (UMBmark rút gọn): suy ra hệ số đường kính trung bình và tỉ lệ hai bánh.
- **Quay 360° ×N**: suy ra track width hiệu dụng (so với gyro).
- **ToF**: đặt tấm phẳng ở 100/200/300 mm để lấy offset và scale từng cảm biến, cùng offset góc lắp.
- **Động cơ**: quét PWM để tìm vùng chết `ks` và `kv`.

## 7. Kế hoạch kiểm chứng
1. Unit test lõi (kinematics, path param, profile, KF) trên `native`.
2. Sim 2 xe: mô hình động cơ bậc 1, trượt bánh, nhiễu encoder/gyro, mô hình ToF (nhiễu, FOV, mất mép tấm), độ trễ và mất gói ESP-NOW. Chạy bộ đường mẫu và chấm điểm N1–N3.
3. Phần cứng từng bước: bánh, rồi 1 xe chạy đường, rồi follower bám trên đường thẳng, rồi qua góc cua.

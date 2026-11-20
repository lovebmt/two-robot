# Động cơ mạnh yếu và ma sát khác nhau

Nguồn: [đợt 3](../raw/2026-11-20-answers-3.md). Bổ sung cho [algorithms §1](algorithms.md#1-động-học-và-điều-khiển-bánh).

## 1. Nguồn gây sai lệch
| # | Hiện tượng | Hậu quả nếu không xử lý |
|---|---|---|
| S1 | 2 motor trên cùng một xe mạnh yếu khác nhau (cuộn dây, hộp số, ma sát hộp số) | Xe đi cong. Quay tại chỗ bị lệch tâm |
| S2 | Cùng một motor nhưng chạy tiến và lùi khác nhau | Quay trái và quay phải cho kết quả khác nhau |
| S3 | Hai xe khác nhau về tốc độ tối đa | Follower yếu hơn sẽ không đuổi kịp leader |
| S4 | Ma sát sàn thay đổi theo vị trí (gạch, thảm, mối nối), caster bị kẹt | Tốc độ dao động, xe bị kéo lệch hướng |
| S5 | Pin tụt áp trong lúc chạy (3S: 12.6 V xuống 10.5 V) | Cùng một duty nhưng tốc độ giảm khoảng 20 % |
| S6 | Vùng chết và ma sát tĩnh ở tốc độ thấp. L298N còn làm mất mô-men ở duty thấp | Xe giật cục hoặc không nhích được khi tiến chậm vào góc cua và khi căn thẳng (ALIGN) |
| S7 | Trượt bánh khi quay hoặc tăng tốc gấp | Encoder đếm sai quãng đường thật |

## 2. Giải pháp phân tầng (từ trong ra ngoài)

```
 v,ω mong muốn ─► [L3 bão hoà ưu tiên ω] ─► v_l*, v_r* ─► [L1 FF(LUT, V_bat) + PI bánh] ─► duty ─► L298N
        ▲                                                         ▲ encoder (đo theo chu kỳ khi chậm)
        └── [L2 vòng hướng: gyro + đồng bộ quãng 2 bánh] ◄────────┘
 [L4 học thích nghi FF]   [L5 xử lý tốc độ thấp]   [L6 giới hạn tốc độ giữa 2 xe]   [L7 phát hiện stall/slip]
```

**L0 — Hiệu chuẩn riêng từng bánh, từng chiều** (xử lý S1, S2)
- Bài *quét PWM*: tăng duty theo bậc, đo tốc độ ổn định. Lưu **bảng tra (LUT) 12 điểm `điện áp → tốc độ`** cho mỗi bánh và mỗi chiều (4 bảng). Bảng tra tốt hơn mô hình tuyến tính `ks + kv·v` vì L298N phi tuyến.
- Lưu theo **điện áp** chứ không theo duty: `duty = V_cần / (V_bat − V_drop)`. Nhờ vậy tự bù được pin tụt áp (S5).
- Bài *step response* trên từng bánh: fit mô hình bậc 1 với `K` (tốc độ trên mỗi volt) và `τ` (hằng số thời gian). Từ đó tự tính hệ số PI theo lambda tuning: `Kp = τ/(K·λ)`, `Ki = Kp/τ`, chọn `λ ≈ τ/2`.
  **Các tham số này cũng được nạp ngược vào mô phỏng**, để mô phỏng giống xe thật ([simulation §3](simulation.md#3-mô-hình-vật-lý)).

**L1 — Vòng tốc độ riêng từng bánh, 100 Hz** (xử lý S1, S2, S4, S5)
- `duty = FF_LUT(v*, chiều, V_bat) + PI(v* − v_đo)`, có anti-windup.
- Phần FF làm khoảng 90 % công việc. Phần I bù chỗ còn lại: motor yếu hơn, ma sát tăng, pin tụt.
- Đo tốc độ chính xác ở tốc độ thấp. Ví dụ hộp số 1:34, bánh 65 mm thì khoảng 7.3 tick/mm. Ở 40 mm/s, mỗi chu kỳ 10 ms chỉ có ~3 tick, tức sai số lượng tử ~35 %.
  Cách xử lý: **đo chu kỳ giữa 2 cạnh xung** (có timestamp) khi chậm, đo **đếm tick trên cửa sổ 50 ms** khi nhanh, rồi lọc nhẹ.

**L2 — Vòng hướng ở mức cả xe** (xử lý phần S1/S4 còn sót lại sau L1)
- Gyro giữ hướng `ψ_seg` cộng PI đồng bộ quãng đường 2 bánh. Đầu ra là **hiệu chỉnh ω**, không chỉnh trực tiếp từng bánh.
- Ví dụ caster bị kẹt kéo xe lệch: gyro thấy lệch hướng thì ω được bù ngay, dù vòng tốc độ của 2 bánh vẫn đang "đúng".

**L3 — Xử lý bão hoà, ưu tiên ω** (S1, S3)
- Nếu `v_l*` hoặc `v_r*` vượt khả năng của bánh yếu hơn (theo LUT ở `V_bat` hiện tại), thì **giảm v và giữ nguyên ω**: xe chạy chậm lại nhưng vẫn đi thẳng hoặc quay đúng.
- Bật cờ `SAT` trong telemetry. Bão hoà kéo dài thì gửi cảnh báo lên PC.

**L4 — Thích nghi trực tuyến** (S4, S5, motor lão hoá)
- Khi chạy ổn định, chuyển dần phần I sang offset của LUT (tích phân rất chậm, có giới hạn ±20 %). Nhờ vậy phần I luôn quanh 0 và sẵn sàng bù nhiễu mới.
- Tuỳ chọn lưu offset đã học vào flash sau mỗi lần chạy (nút "Lưu hiệu chỉnh học được").

**L5 — Tốc độ thấp và ma sát tĩnh** (S6)
- `v_min_reliable` lấy từ bài quét PWM. Mọi lệnh tốc độ nhỏ hơn mức này được thay bằng **điều khiển vị trí** kèm **xung khởi động** (kick ~30 ms ở duty cao để vượt ma sát tĩnh rồi hạ xuống).
- Tần số PWM của L298N chỉnh được (500 Hz–5 kHz). Tần số thấp cho mô-men tốt hơn ở duty thấp. Dùng bài quét để chọn.
- Quay tại chỗ: 2 bánh có profile riêng nhưng được đồng bộ theo góc gyro. Nếu một bánh đứng (ma sát tĩnh) thì bánh kia giảm theo, tránh xe quay quanh 1 bánh.

**L6 — Phối hợp giữa 2 xe** (S3)
- Web tính `v_cruise ≤ headroom × min(v_max_leader, v_max_follower)`, với `headroom ≈ 0.7`. Follower **phải có dư tốc độ** để đuổi kịp gap.
- `v_max` của mỗi xe lấy từ LUT ở `V_bat` thấp nhất cho phép. Validator cảnh báo nếu người dùng đặt `v_cruise` quá cao.
- Tương tự với `v_corner` và tốc độ quay: lấy theo xe chậm hơn.

**L7 — Phát hiện bất thường** (S6, S7)
- **Stall:** duty > ngưỡng mà không có tick trong `stall_ms` thì phanh và báo `STALL`.
- **Trượt:** `|ω_enc − ω_gyro|` lớn thì bật cờ `SLIP`. Lúc đó fusion hướng chuyển sang chỉ tin gyro, và quãng đường đo được kém tin cậy (follower dựa vào ToF nhiều hơn).
- **Lệch motor lớn:** offset học được ở L4 vượt ±20 % thì cảnh báo kiểm tra cơ khí.

## 3. Hiệu chuẩn trên web (wizard)
| Bước | Kết quả lưu vào flash |
|---|---|
| Kiểm tra chiều motor và encoder | `motor.invert_*`, `enc.invert_*` |
| Quét PWM (tiến và lùi, từng bánh) | `motor.lut_{l,r}_{fwd,rev}`, `v_min_reliable`, `v_max` |
| Step response | `motor.model_{l,r}.{K,tau}` và PI tự tính |
| Đi thẳng 1 m | `wheel.diam_eps`, scale đường kính |
| Quay 4×90° và 360° | `imu.gyro_scale_z`, `wheel.track_eff_mm` |

Mỗi bước có biểu đồ (duty, tốc độ đo, mô hình fit) để người dùng xem trước khi bấm Apply rồi Save.

## 4. Kiểm chứng trong mô phỏng
Mỗi kịch bản trong [simulation.md](simulation.md) có biến thể ngẫu nhiên: lệch motor ±20 %, ma sát sàn ±30 % theo vùng, pin từ 12.6 V xuống 10.8 V, caster kẹt ngắn. Tiêu chí: vẫn đạt N1–N3.

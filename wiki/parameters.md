# Thông số cấu hình (Web Config)

Đơn vị lưu nội bộ là **mm, mm/s, rad**. UI có thể hiển thị cm/độ. Mỗi xe có một bản cấu hình (có thể copy từ xe kia). Giá trị mặc định chỉ là ví dụ.

## Bản đồ
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `grid.cell_mm` | kích thước thực của 1 ô lưới | 100 |
| `grid.cols`, `grid.rows` | số ô | 30 × 20 |
| `path.start` | ô xuất phát và hướng (0/90/180/270) | — |

## Hình học xe (hình hộp)
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `body.length_mm`, `body.width_mm`, `body.height_mm` | kích thước thân | 160 × 120 × 70 |
| `body.axle_to_front_mm` | tâm trục bánh tới mặt trước | 120 |
| `body.axle_to_rear_mm` | tâm trục bánh tới mặt sau (`x_rear`) | 40 |
| `body.sweep_radius_mm` | bán kính quét khi quay tại chỗ (tính tự động, có thể ghi đè) | auto |
| `wheel.diameter_mm` | đường kính bánh dẫn động | 65 |
| `wheel.track_mm` | khoảng cách 2 bánh (tâm–tâm vệt bánh) | 130 |
| `wheel.diam_ratio_lr` | hệ số bù đường kính trái/phải | 1.000 |
| `caster.offset_mm` | vị trí bánh caster so với trục | 100 |

## Động cơ và encoder
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `enc.cpr` | xung/vòng trục động cơ (1 kênh) | 7 |
| `enc.gear_ratio` | tỉ số hộp số | 100 |
| `enc.invert_l/r`, `motor.invert_l/r` | đảo chiều | false |
| `motor.pwm_hz`, `motor.deadband` (`ks`), `motor.kv` | đặc tính | 20000, —, — |
| `motor.v_max_mm_s` | vận tốc tối đa vật lý | 400 |

## Cảm biến khoảng cách
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `tof.type` | VL53L0X / VL53L1X / … | VL53L1X |
| `tof.x_mm` | tâm trục tới mắt cảm biến (dọc) | 120 |
| `tof.baseline_mm` | khoảng cách giữa 2 mắt cảm biến (ngang) | 80 |
| `tof.toe_deg` | góc nghiêng ra ngoài của mỗi cảm biến | 0 |
| `tof.offset_l/r_mm`, `tof.scale_l/r` | hiệu chuẩn | 0, 1 |
| `tof.rate_hz`, `tof.timing_budget_ms` | tần số / thời gian đo | 33, 30 |
| `tof.min/max_valid_mm` | khoảng hợp lệ | 30 / 1200 |
| `target.rear_plate_width_mm` | bề rộng tấm phản xạ đuôi leader | 120 |

## IMU
`imu.type`, `imu.gyro_bias_auto` (true), `imu.yaw_fusion_alpha`.

## Chuyển động và profile
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `motion.v_cruise` | vận tốc trên đoạn thẳng | 200 |
| `motion.a_max`, `motion.j_max` | gia tốc / jerk | 300, 1500 |
| `motion.v_corner` | vận tốc trong vùng tiếp cận góc | 50 |
| `motion.approach_dist` | bắt đầu giảm tốc trước góc | 150 |
| `motion.turn_omega_max` | tốc độ quay tại chỗ (deg/s) | 90 |
| `motion.turn_tol_deg` | sai số chấp nhận khi quay | 1.0 |
| `motion.exit_dist` | leader tiến thêm sau khi quay | auto (≥ clear_dist) |

## Đội hình (formation)
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `gap.target_mm` | gap tâm–tâm mong muốn trên đoạn thẳng | 300 |
| `gap.min_safe_mm` | dưới ngưỡng này thì dừng | 200 |
| `gap.max_mm` | leader chờ nếu vượt ngưỡng | 600 |
| `gap.hold_dist_mm` | follower đứng cách góc khi leader quay | auto |
| `gap.lost_timeout_ms` | mất leader thì dừng | 500 |

## Bộ điều khiển và bộ lọc
`pid.wheel.{kp,ki,kff}`, `track.{k_psi,k_y,lookahead}`, `follow.{kp,ki}`, `kf.{q_a, r_tof, r_radio_s, r_radio_v, gate_chi2}`.

## Mạng và an toàn
`net.hostname`, `net.link_timeout_ms` (500), `net.telemetry_hz` (10), `peer.rate_hz` (50), `peer.channel`, `safety.bat_min_v` (6.6).

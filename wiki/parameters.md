# Thông số cấu hình (Web Config)

Đơn vị nội bộ: **mm, mm/s, rad** (UI hiển thị cm/độ). Mỗi xe một profile. Giá trị mặc định chỉ là ví dụ, sẽ thay bằng số thật từ [hardware-intake](../work/hardware/hardware-intake.md).
`auto` nghĩa là giá trị tính từ các thông số khác (vẫn ghi đè được).
Tất cả khoá dưới đây **nhập trên web, gửi xuống xe, lưu flash** ([config-management.md](config-management.md)). Nguồn chuẩn là `work/shared/config.schema.json`.
Quy ước áp dụng: nhóm `hw.*` cần **reboot**; nhóm hình học và cảm biến áp dụng khi **idle**; hệ số điều khiển áp dụng **live**.

## Phần cứng / GPIO (`hw.*`, reboot)
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `hw.pins.ena`, `in1`, `in2`, `enb`, `in3`, `in4` | L298N | 25, 26, 27, 14, 23, 5 |
| `hw.pins.enc_l_a/b`, `enc_r_a/b` | encoder | 34, 35, 36, 39 |
| `hw.pins.i2c0_sda/scl`, `tof_xshut_l/r`, `tof_int_l/r` | ToF | 21, 22, 18, 19, 16, 17 |
| `hw.pins.i2c1_sda/scl`, `imu_int` | MPU6050 | 13, 4, 33 |
| `hw.pins.vbat_adc`, `led`, `buzzer` | | 32, 2, 15 |
| `hw.vbat_divider` | hệ số cầu chia áp pin | ? |
| `hw.l298n_vdrop_v` | sụt áp ước lượng | 2.0 |

## Bản đồ / project
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `grid.cell_mm` | kích thước thực 1 ô | 100 |
| `grid.cols`, `grid.rows` | số ô | 30 × 20 |

## Hình học xe (hình hộp)
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `body.length_mm`, `body.width_mm`, `body.height_mm` | kích thước thân | 160 × 120 × 70 |
| `body.axle_to_front_mm` | tâm trục bánh tới mặt trước | 120 |
| `body.axle_to_rear_mm` | tâm trục tới mặt sau / tấm phản xạ (`x_rear`) | 40 |
| `body.sweep_radius_mm` | `R_sw` | auto |
| `wheel.diameter_mm` | đường kính bánh dẫn động | 65 |
| `wheel.track_mm` | khoảng cách 2 bánh (đo bằng thước) | 130 |
| `wheel.track_eff_mm` | `T_eff` sau hiệu chuẩn quay | auto = track |
| `wheel.diam_eps` | `ε` bù trái/phải | 0.000 |
| `caster.offset_mm` | vị trí caster so với trục | 100 |
| `target.plate_width_mm` | bề rộng tấm phản xạ đuôi | 120 |

## Driver L298N và motor
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `motor.pwm_hz`, `motor.pwm_bits` | LEDC (reboot) | 2000, 10 |
| `motor.rated_v` | điện áp định mức motor | 12 |
| `motor.invert_l/r` | đảo chiều motor | false |
| `motor.lut_{l,r}_{fwd,rev}` | bảng tra 12 điểm `[V, mm/s]` | hiệu chuẩn |
| `motor.model_{l,r}.{K,tau}` | mô hình bậc 1 (từ step response, dùng cả cho sim) | hiệu chuẩn |
| `motor.v_min_reliable` | dưới mức này chuyển sang điều khiển vị trí + kick | hiệu chuẩn |
| `motor.kick_duty`, `motor.kick_ms` | xung vượt ma sát tĩnh | 0.6, 30 |
| `motor.v_max_mm_s` | tốc độ tối đa ở `V_bat` thấp nhất | auto |
| `motor.adapt.enable`, `motor.adapt.rate`, `motor.adapt.limit` | thích nghi FF (L4) | true, 0.01, 0.2 |
| `motion.headroom` | `v_cruise ≤ headroom·min(v_max 2 xe)` | 0.7 |
| `motor.brake_on_stop` | phanh nhanh khi dừng | true |
| `motor.stall_ms` | duty > ngưỡng mà không có tick thì báo STALL | 500 |

## Encoder
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `enc.ppr_motor` | xung/vòng trục motor, 1 kênh (GA25-370 thường là 11) | 11 |
| `enc.gear_ratio` | tỉ số hộp số GA25-370 | ? (intake) |
| `enc.invert_l/r` | đảo chiều đếm | false |
| `enc.glitch_ns` | lọc nhiễu PCNT | 1000 |
| `enc.mm_per_tick` | | auto |

## Cảm biến khoảng cách
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `range.type` | ToF200C (nhân VL53L0X) | `vl53l0x` |
| `range.addr_l`, `range.addr_r` | địa chỉ I²C sau khi đổi qua XSHUT | `0x30`, `0x29` |
| `range.timing_budget_ms` | 20 (nhanh) / 33 (mặc định) / 66 (chính xác) | 33 |
| `range.toe_in_deg` | góc nghiêng vào trong của mỗi cảm biến | 0 |
| `range.x_mm` | tâm trục tới mắt cảm biến (`x_s`) | 120 |
| `range.baseline_mm` | khoảng cách 2 mắt (`b`) | 60 |
| `range.offset_{l,r}_mm`, `range.scale_{l,r}` | hiệu chuẩn | 0, 1 |
| `range.rate_hz` | tần số (mỗi cảm biến) | 30 |
| `range.min_valid_mm`, `range.max_valid_mm` | vùng hợp lệ | 30, 1000 |
| `range.max_diff_mm` | `|d_l − d_r|` tối đa còn tin | 40 |
| `range.sigma_mm` | độ lệch chuẩn nhiễu (cho KF) | 6 |

## IMU MPU6050
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `imu.i2c_bus` | 0 (chung với ToF) / 1 (bus riêng) | 1 |
| `imu.addr` | | `0x68` |
| `imu.gyro_range_dps`, `imu.dlpf_hz`, `imu.rate_hz` | | 250, 42, 200 |
| `imu.gyro_bias_z`, `imu.gyro_scale_z` | hiệu chuẩn | auto, 1.0 |
| `imu.mount_sign_z` | +1/−1 tuỳ chiều gắn | +1 |
| `imu.fusion_beta_straight`, `imu.fusion_beta_turn` | trọng số encoder trong fusion hướng | 0.05, 0.0 |
| `imu.zupt_enable` | cập nhật bias khi đứng yên | true |
| `motion.turn_tol_deg` | sai số chấp nhận khi quay theo gyro | 1.0 |
| `motion.turn_omega_max_dps` | | 90 |
| `pid.heading.{kp,ki}` | giữ hướng khi đi thẳng | — |

## Chuyển động
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `motion.v_cruise` | tốc độ đoạn thẳng | 150 |
| `motion.a_max` | gia tốc | 250 |
| `motion.v_corner` | tốc độ khi tiếp cận góc | 40 |
| `motion.approach_dist` | bắt đầu giảm tốc trước góc | 150 |
| `motion.stop_tol_mm` | sai số dừng tại điểm | 3 |
| `motion.turn_wheel_v` | tốc độ bánh khi quay tại chỗ | 60 |
| `motion.turn_a_max` | gia tốc bánh khi quay | 150 |
| `motion.obstacle_stop_mm` | leader phanh khi thấy vật | 80 |

## Đội hình và góc cua
| Khoá | Mô tả | Mặc định |
|---|---|---|
| `gap.target_mm` | gap tâm–tâm trên đoạn thẳng (`g*`) | 300 |
| `gap.min_safe_mm` | dưới ngưỡng thì phanh | auto |
| `gap.hold_mm` | follower chờ khi leader quay (`g_hold`) | auto (≥ `g_hold_min`) |
| `corner.exit_dist_mm` | leader tiến thêm sau khi quay | auto (≥ `exit_dist_min`) |
| `corner.margin_mm` | biên an toàn | 30 |
| `corner.snapshot_n`, `corner.snap_std_max_mm` | số mẫu, ngưỡng nhiễu | 15, 4 |
| `corner.align_tol_deg` | ngưỡng căn song song | 1.5 |
| `corner.reacquire_tol_mm`, `corner.scan_deg` | | 40, 10 |
| `corner.step_timeout_ms` | timeout mỗi bước | 8000 |
| `gap.lost_timeout_ms` | mất leader trên đoạn thẳng | 400 |

## Bộ điều khiển và bộ lọc
`pid.wheel.{kp,ki}`, `pid.straight.{kp,ki}` (đồng bộ 2 bánh), `pid.pos.kp` (dừng tại điểm),
`follow.{kp_g, ki_g, k_psi, k_edge}`, `kf.{q_a, r_relay_v, gate_chi2}`, `filt.dpsi_alpha`.

## Mạng và an toàn
`net.hostname`, `net.link_timeout_ms` (300), `net.hb_hz` (5), `net.state_hz` (25), `net.udp_state_port` (47801), `net.udp_relay_port` (47802), `safety.bat_min_mv` (10500, 3S), `safety.allow_tune_while_running` (true).

## Meta (không chỉnh tay)
`cfg.schema_ver`, `cfg.rev`, `cfg.crc32`, `cfg.robot_id`.

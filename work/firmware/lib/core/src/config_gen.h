// AUTO-GENERATED từ work/shared/config.schema.json bởi work/shared/codegen_cpp.py — KHÔNG SỬA TAY
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace tr {

constexpr int SCHEMA_VER = 1;

enum class PType : uint8_t { Float, Int, Bool };

// Bit nhóm tham số (ChangeSet)
constexpr uint32_t G_BODY = 1u << 0;
constexpr uint32_t G_WHEEL = 1u << 1;
constexpr uint32_t G_ENC = 1u << 2;
constexpr uint32_t G_MOTOR = 1u << 3;
constexpr uint32_t G_POWER = 1u << 4;
constexpr uint32_t G_RANGE = 1u << 5;
constexpr uint32_t G_IMU = 1u << 6;
constexpr uint32_t G_PID = 1u << 7;
constexpr uint32_t G_FOLLOW = 1u << 8;
constexpr uint32_t G_PLATE = 1u << 9;
constexpr uint32_t G_MOTION = 1u << 10;
constexpr uint32_t G_GAP = 1u << 11;
constexpr uint32_t G_CORNER = 1u << 12;
constexpr uint32_t G_START = 1u << 13;
constexpr uint32_t G_SAFETY = 1u << 14;

struct Config {
  float body_length_mm = 160;  // mm Chiều dài thân
  float body_width_mm = 120;  // mm Chiều rộng thân
  float body_axle_to_front_mm = 120;  // mm Tâm trục bánh tới mặt trước
  float body_axle_to_rear_mm = 40;  // mm Tâm trục bánh tới mặt sau / tấm phản xạ
  float body_plate_width_mm = 120;  // mm Bề rộng tấm phản xạ đuôi
  float wheel_diameter_mm = 65;  // mm Đường kính bánh dẫn động
  float wheel_track_mm = 130;  // mm Khoảng cách 2 bánh (đo thước)
  float wheel_track_eff_mm = 130;  // mm Track hiệu dụng (sau hiệu chuẩn quay)
  float wheel_diam_eps = 0.0f;  //  Bù đường kính trái/phải: D_l=D(1+e), D_r=D(1-e)
  int32_t enc_ppr_motor = 11;  // xung/vòng Xung/vòng trục motor (1 kênh)
  float enc_gear_ratio = 20.239f;  //  Tỉ số hộp số GA25-370
  bool enc_invert_l = false;  //  Đảo chiều đếm encoder trái (xe tiến mà đếm âm)
  bool enc_invert_r = true;  //  Đảo chiều đếm encoder phải
  float motor_ks_l_v = 1.2f;  // V Vùng chết bánh trái (điện áp)
  float motor_ks_r_v = 1.2f;  // V Vùng chết bánh phải (điện áp)
  float motor_kbreak_l_v = 0;  // V Điện áp bứt ma sát tĩnh bánh trái (lúc bánh đứng yên; 0 = dùng vùng chết)
  float motor_kbreak_r_v = 0;  // V Điện áp bứt ma sát tĩnh bánh phải
  float motor_kv_l = 0.0126f;  // V/(mm/s) Điện áp trên mỗi mm/s, bánh trái
  float motor_kv_r = 0.0126f;  // V/(mm/s) Điện áp trên mỗi mm/s, bánh phải
  float motor_v_min_reliable = 15;  // mm/s Tốc độ bánh tối thiểu điều khiển được
  float motor_adapt_rate = 0.05f;  // 1/s Tốc độ học offset FF (L4), 0 = tắt
  float motor_adapt_limit = 0.2f;  //  Giới hạn offset học được (tỉ lệ)
  bool motor_invert_l = true;  //  Đảo chiều motor trái (lệnh tiến mà bánh quay lùi)
  bool motor_invert_r = true;  //  Đảo chiều motor phải
  int32_t motor_pwm_hz = 2000;  // Hz Tần số PWM L298N (thấp → mô-men tốt hơn ở duty thấp)
  float power_l298n_vdrop_v = 2.0f;  // V Sụt áp L298N
  float power_bat_min_v = 10.5f;  // V Ngưỡng pin yếu (3S)
  float power_vbat_divider = 0;  //  Hệ số cầu chia đo pin (V_pin = V_adc × hệ số). 0 = không đo pin
  float range_x_mm = 120;  // mm Tâm trục tới mắt cảm biến (dọc)
  float range_baseline_mm = 60;  // mm Khoảng cách 2 mắt ToF (ngang)
  float range_offset_l_mm = 0;  // mm Offset hiệu chuẩn ToF trái
  float range_offset_r_mm = 0;  // mm Offset hiệu chuẩn ToF phải
  float range_min_valid_mm = 30;  // mm Khoảng đo hợp lệ tối thiểu
  float range_max_valid_mm = 1200;  // mm Khoảng đo hợp lệ tối đa
  float range_min_signal_mcps = 1.5f;  // MCPS Cường độ tín hiệu phản hồi tối thiểu để tin số đo ToF (loại số ma khi phía trước trống)
  float range_max_diff_mm = 40;  // mm |d_l - d_r| tối đa còn tin
  float range_sigma_mm = 6;  // mm Độ lệch chuẩn nhiễu ToF (cho KF)
  float imu_gyro_scale_z = 1.0f;  //  Hệ số scale gyro Z
  bool imu_invert_z = false;  //  Đảo dấu gyro Z (MPU6050 gắn úp ngược)
  float imu_beta_straight = 0.05f;  //  Trọng số encoder trong fusion hướng khi đi thẳng
  bool imu_auto_calib = true;  //  Tự calib gyro khi khởi động (xe đứng yên)
  bool imu_require_calib = true;  //  Từ chối ARM nếu gyro chưa calib
  bool imu_recal_before_run = true;  //  Calib lại gyro ngay trước mỗi lần chạy
  float imu_calib_time_s = 2.0f;  // s Thời gian đứng yên để calib
  float imu_calib_max_spread_dps = 1.5f;  // °/s Dao động gyro tối đa khi calib (lớn hơn → coi là bị chạm, calib lại)
  float imu_bias_max_dps = 10;  // °/s Bias lớn hơn mức này → cảnh báo IMU bất thường
  bool imu_zupt_enable = true;  //  Cập nhật bias mỗi khi xe đứng yên (ZUPT)
  float imu_zupt_gain = 0.01f;  //  Hệ số lọc bias khi ZUPT (mỗi chu kỳ 10 ms)
  bool imu_temp_comp = true;  //  Bù bias theo nhiệt độ chip khi đang chạy (học hệ số từ các lần đứng yên)
  float imu_temp_coeff_dps_c = 0.0f;  // °/s/°C Hệ số bias theo nhiệt độ (giá trị khởi đầu; robot tự học, lưu lại sau khi chạy)
  float imu_still_gyro_dps = 2.0f;  // °/s |ω gyro| lớn hơn → không coi là đứng yên
  float pid_wheel_kp = 0.01f;  // V/(mm/s) PI tốc độ bánh: Kp
  float pid_wheel_ki = 0.15f;  // V/mm PI tốc độ bánh: Ki
  float pid_heading_kp = 4.0f;  // 1/s Giữ hướng: Kp
  float pid_heading_ki = 0.5f;  // 1/s² Giữ hướng: Ki
  float pid_pos_kp = 2.5f;  // 1/s Dừng tại điểm: Kp
  float follow_kp_g = 1.2f;  // 1/s Bám gap: Kp
  float follow_ki_g = 0.1f;  // 1/s² Bám gap: Ki
  float follow_k_psi = 2.0f;  // 1/s Căn song song theo Δψ
  float follow_k_edge = 0.15f;  // rad/s Lái khi 1 chùm mất mép tấm
  float plate_k_psi = 3.0f;  // 1/s Bám tấm: xoay theo góc tấm (Δψ) — lớn = bám cua nhanh, quá lớn = lắc
  float plate_k_edge = 0.8f;  // rad/s Bám tấm: tốc độ xoay về phía mắt còn thấy tấm khi 1 mắt mất tấm
  float plate_max_dpsi_deg = 50;  // ° Bám tấm: góc tấm lớn nhất còn tin (2 mắt cùng trúng tấm)
  float plate_dist_mm = 200;  // mm Khoảng cách giữ: mặt kính ToF → tấm
  float plate_v_max = 150;  // mm/s Tốc độ tiến tối đa khi bám tấm
  float plate_v_back = 80;  // mm/s Tốc độ lùi tối đa khi tấm lại quá gần (0 = không lùi)
  float plate_hold_band_mm = 10;  // mm Vùng chết quanh khoảng cách giữ: trong vùng này xe đứng yên (chống giật tới-lui)
  float plate_kp = 1.2f;  // 1/s Độ cứng bám khoảng cách: lớn = đuổi nhanh, quá lớn = dao động tới-lui
  float follow_kf_q_a = 300;  // mm/s² KF: nhiễu gia tốc leader
  float follow_kf_gate_chi2 = 9;  //  KF: ngưỡng gating
  int32_t follow_lost_timeout_ms = 500;  // ms Mất leader quá thời gian này → dừng
  float follow_v_headroom = 1.4f;  //  Follower được chạy tới v_cruise × hệ số này để đuổi
  float motion_v_cruise = 150;  // mm/s Tốc độ đoạn thẳng
  float motion_a_max = 250;  // mm/s² Gia tốc tối đa
  float motion_v_corner = 40;  // mm/s Tốc độ khi tiếp cận điểm dừng
  float motion_approach_dist = 120;  // mm Bắt đầu chạy chậm trước điểm dừng
  float motion_stop_tol_mm = 2;  // mm Sai số dừng tại điểm
  float motion_turn_rate_dps = 90;  // °/s Tốc độ quay tại chỗ tối đa
  float motion_turn_acc_dps2 = 240;  // °/s² Gia tốc góc khi quay
  float motion_settle_tol_mm = 6;  // mm Xe kẹt (đứng yên) trong khoảng này trước điểm dừng → coi như tới
  float motion_settle_tol_deg = 2;  // ° Xe kẹt (đứng yên) trong khoảng này trước góc đích → coi như tới
  float motion_stall_time_s = 0.6f;  // s Đứng yên bao lâu thì coi là kẹt (áp dụng 2 ngưỡng trên)
  float motion_turn_tol_deg = 0.8f;  // ° Sai số chấp nhận khi quay
  float gap_target_mm = 400;  // mm Gap tâm–tâm mong muốn trên đoạn thẳng
  float gap_hold_mm = 0;  // mm Gap chờ khi leader quay (0 = tự tính)
  float corner_exit_dist_mm = 0;  // mm Leader tiến thêm sau khi quay (0 = tự tính)
  float corner_follow_start_mm = 40;  // mm Leader ra khỏi góc bấy nhiêu thì follower bắt đầu tiến
  float corner_margin_mm = 30;  // mm Biên an toàn
  int32_t corner_snapshot_n = 12;  // mẫu Số mẫu ToF khi SNAPSHOT
  float corner_align_tol_deg = 1.5f;  // ° Ngưỡng căn song song
  float corner_reacquire_tol_mm = 60;  // mm Sai lệch cho phép khi bắt lại leader
  float corner_scan_deg = 12;  // ° Góc quét khi không thấy leader
  float corner_step_timeout_s = 20;  // s Timeout mỗi bước
  bool start_check_enable = true;  //  Bắt buộc kiểm tra vị trí 2 xe trước khi chạy
  float start_gap_tol_mm = 40;  // mm Gap lúc xuất phát được lệch so với gap mục tiêu
  float start_max_dpsi_deg = 6;  // ° Xe sau lệch góc quá mức này → yêu cầu đặt lại
  float start_align_dpsi_deg = 1.5f;  // ° Xe sau lệch góc quá mức này → tự ALIGN trước khi chạy
  float start_front_clear_mm = 250;  // mm Phía trước xe đầu phải trống ít nhất bấy nhiêu
  bool safety_allow_tune_while_running = true;  //  Cho chỉnh tham số live khi đang chạy
  int32_t safety_link_timeout_ms = 1500;  // ms Mất heartbeat PC → phanh
  float safety_tof_anomaly_jump_mm = 120;  // mm Một mắt ToF ngắn đột ngột quá ngưỡng → dừng
  int32_t safety_tof_anomaly_samples = 2;  // mẫu Số mẫu bất thường liên tiếp để báo vật cản
};

struct ParamMeta {
  const char* key;
  PType type;
  uint16_t offset;
  float min;
  float max;
  uint32_t group;
  bool idle;    // chỉ đổi khi xe đứng yên
  bool shared;  // tham số chung 2 xe
};

extern const ParamMeta PARAMS[];
extern const size_t PARAM_COUNT;
extern const char* const GROUP_NAMES[];
extern const size_t GROUP_COUNT;

}  // namespace tr

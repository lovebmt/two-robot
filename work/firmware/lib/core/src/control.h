#pragma once
// Điều khiển bánh, động học, calib gyro, ước lượng hướng, odometry.
// Tương ứng tworobot/fw/control.py (wiki/motor-control.md, wiki/algorithms.md §2).
#include <stdint.h>

#include "config_gen.h"

namespace tr {

// FF (vùng chết + kv, bù V_bat) + PI tốc độ + học offset FF. Tích phân lưu theo volt → bumpless.
struct WheelController {
  char side;
  float integ = 0, adapt = 0, vref = 0, duty = 0;
  explicit WheelController(char s) : side(s) {}
  void reset() { integ = 0; vref = 0; duty = 0; }
  float update(float v_ref, float v_meas, float vbat, const Config& c, float dt);
};

// (v, ω) → tốc độ 2 bánh → PWM. Đo tốc độ bánh từ encoder (cửa sổ trượt 50 ms).
struct DriveBase {
  static constexpr int WINDOW = 5;
  WheelController wl{'l'}, wr{'r'};
  float hl[WINDOW] = {0}, hr[WINDOW] = {0};
  int hn = 0, hi = 0;
  float v_meas_l = 0, v_meas_r = 0;
  bool sat = false, braking = true;
  float vMeas() const { return 0.5f * (v_meas_l + v_meas_r); }
  void measure(float dl, float dr, float dt);
  // trả duty trái/phải và brake
  void command(float v, float w, bool brake, const Config& c, float vbat, float dt, float& dl, float& dr, bool& b);
};

struct CalibResult {
  bool ok = false;
  const char* err = nullptr;
  float bias = 0, stdev = 0, spread = 0;
  int n = 0, restarts = 0;
};

// Calib bias gyro khi đứng yên (algorithms §2.1).
struct ImuCalibrator {
  static constexpr int MAX_RESTART = 5;
  float dur, max_spread;
  int restarts = 0, moved_restarts = 0, n = 0;
  double s = 0, s2 = 0;
  float t = 0, lo = 1e9f, hi = -1e9f;
  explicit ImuCalibrator(const Config& c);
  float progress() const { return t / dur > 1.f ? 1.f : t / dur; }
  // trả true khi xong (thành công hoặc lỗi), kết quả trong out
  bool feed(float gz_raw, bool moved, float dt, CalibResult& out);
 private:
  void reset_() { n = 0; s = s2 = 0; t = 0; lo = 1e9f; hi = -1e9f; }
};

// ψ = ∫[(1-β)ω_gyro + β ω_enc]; bias: calib → ZUPT → bù nhiệt (học k từ các điểm (T, bias)).
struct HeadingEstimator {
  static constexpr float ZUPT_DELAY = 0.3f;
  static constexpr float ANCHOR_MIN_STILL = 1.0f;
  static constexpr float ANCHOR_PERIOD = 5.0f;
  static constexpr int MAX_ANCHORS = 24;
  static constexpr float MIN_TEMP_SPAN = 0.5f;

  float psi = 0, bias = 0, temp = 25, w = 0, w_gyro = 0, w_enc = 0;
  bool zupt_active = false;
  bool has_k = false;
  float k_temp = 0;  // rad/s/°C (học được)
  float still_t = 0, anchor_t = 0;
  float anc_t[MAX_ANCHORS], anc_b[MAX_ANCHORS];
  int anc_n = 0, anc_i = 0;
  bool has_ref = false;
  float ref_t = 0, ref_b = 0;

  float k(const Config& c) const;
  void anchorBias();
  void update(float gz_raw, float dl, float dr, bool straight, bool still, const Config& c, float dt,
              bool calibrating, float temp_c);
};

struct Odometry {
  float x = 0, y = 0, s = 0, arc_l = 0, arc_r = 0;
  bool has_prev = false;
  float psi_prev = 0;
  void reset(float x_, float y_, float s_) { x = x_; y = y_; s = s_; has_prev = false; }
  void update(float dl, float dr, float psi);
};

}  // namespace tr

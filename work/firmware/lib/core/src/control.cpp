#include "control.h"

#include <algorithm>
#include <cmath>

#include "mathutil.h"

namespace tr {

float WheelController::update(float v_ref, float v_meas, float vbat, const Config& c, float dt) {
  vref = v_ref;
  if (v_ref == 0.f) { integ = 0; duty = 0; stall_t = 0; boost = 0; return 0; }
  const float ks = side == 'l' ? c.motor_ks_l_v : c.motor_ks_r_v;
  const float kv = side == 'l' ? c.motor_kv_l : c.motor_kv_r;
  // Ma sát tĩnh > ma sát động: bánh đang đứng yên thì cần điện áp bứt (kbreak) mới nhúc nhích
  const bool stopped = std::fabs(v_meas) < 0.3f * c.motor_v_min_reliable;
  const float kb = std::max(ks, side == 'l' ? c.motor_kbreak_l_v : c.motor_kbreak_r_v);
  const float ff = sgn(v_ref) * ((stopped ? kb : ks) + kv * std::fabs(v_ref)) * (1.f + adapt);
  // Lệnh mà vẫn kẹt > 0.15 s (gần đích, ma sát lớn hơn hiệu chuẩn) → cộng dần 4 V/s; chạy được thì nhả nhanh
  stall_t = stopped ? stall_t + dt : 0.f;
  boost = stall_t > 0.15f ? std::min(boost + 4.f * dt, 4.f) : std::max(0.f, boost - 8.f * dt);
  const float e = v_ref - v_meas;
  integ = clampf(integ + c.pid_wheel_ki * e * dt, -3.f, 3.f);
  const float volts = ff + c.pid_wheel_kp * e + integ + sgn(v_ref) * boost;
  // L4: chuyển dần phần I sang offset FF khi chạy ổn định
  if (c.motor_adapt_rate > 0 && std::fabs(v_ref) > 2.f * c.motor_v_min_reliable && std::fabs(ff) > 0.5f) {
    const float lim = c.motor_adapt_limit;
    adapt = clampf(adapt + c.motor_adapt_rate * dt * clampf(integ * sgn(v_ref) / std::fabs(ff), -1, 1), -lim, lim);
  }
  const float vavail = std::max(vbat - c.power_l298n_vdrop_v, 1.f);
  duty = clampf(volts / vavail, -1.f, 1.f);
  return duty;
}

void DriveBase::measure(float dl, float dr, float dt) {
  hl[hi] = dl;
  hr[hi] = dr;
  hi = (hi + 1) % WINDOW;
  if (hn < WINDOW) ++hn;
  float sl = 0, sr = 0;
  for (int i = 0; i < hn; ++i) { sl += hl[i]; sr += hr[i]; }
  v_meas_l = sl / (hn * dt);
  v_meas_r = sr / (hn * dt);
}

void DriveBase::command(float v, float w, bool brake, const Config& c, float vbat, float dt, float& dl, float& dr,
                        bool& b) {
  braking = brake;
  if (brake) {
    wl.reset();
    wr.reset();
    sat = false;
    dl = dr = 0;
    b = true;
    return;
  }
  const float half_t = c.wheel_track_eff_mm / 2.f;
  const float vavail = vbat - c.power_l298n_vdrop_v;
  const float vmax = 0.95f * std::max(0.f, std::min((vavail - c.motor_ks_l_v) / c.motor_kv_l,
                                                    (vavail - c.motor_ks_r_v) / c.motor_kv_r));
  // L3: bão hoà ưu tiên ω — giảm v, giữ hướng
  float w_part = w * half_t;
  bool s = false;
  if (std::fabs(w_part) > vmax) { w_part = sgn(w_part) * vmax; s = true; }
  const float v_allow = vmax - std::fabs(w_part);
  if (std::fabs(v) > v_allow) { v = sgn(v) * v_allow; s = true; }
  sat = s;
  dl = wl.update(v - w_part, v_meas_l, vbat, c, dt);
  dr = wr.update(v + w_part, v_meas_r, vbat, c, dt);
  b = false;
}

ImuCalibrator::ImuCalibrator(const Config& c)
    : dur(c.imu_calib_time_s), max_spread(deg2rad(c.imu_calib_max_spread_dps)) {}

bool ImuCalibrator::feed(float gz, bool moved, float dt, CalibResult& out) {
  if (moved || (n && (std::max(hi, gz) - std::min(lo, gz)) > max_spread)) {
    ++restarts;
    if (moved) ++moved_restarts;
    reset_();
    if (restarts > MAX_RESTART) {
      out.ok = false;
      // Bánh không hề nhích mà gyro vẫn dao động → gyro nhiễu, không phải bị chạm
      out.err = moved_restarts ? "IMU_NOT_STILL" : "IMU_NOISY";
      out.restarts = restarts;
      return true;
    }
    return false;
  }
  ++n;
  s += gz;
  s2 += static_cast<double>(gz) * gz;
  lo = std::min(lo, gz);
  hi = std::max(hi, gz);
  t += dt;
  if (t < dur) return false;
  const double mean = s / n;
  out.ok = true;
  out.bias = static_cast<float>(mean);
  out.stdev = static_cast<float>(std::sqrt(std::max(0.0, s2 / n - mean * mean)));
  out.spread = hi - lo;
  out.n = n;
  out.restarts = restarts;
  return true;
}

float HeadingEstimator::k(const Config& c) const { return has_k ? k_temp : deg2rad(c.imu_temp_coeff_dps_c); }

void HeadingEstimator::anchorBias() {
  has_ref = true;
  ref_t = temp;
  ref_b = bias;
  anc_t[anc_i] = temp;
  anc_b[anc_i] = bias;
  anc_i = (anc_i + 1) % MAX_ANCHORS;
  if (anc_n < MAX_ANCHORS) ++anc_n;
  float tmin = 1e9f, tmax = -1e9f, mt = 0, mb = 0;
  for (int i = 0; i < anc_n; ++i) {
    tmin = std::min(tmin, anc_t[i]);
    tmax = std::max(tmax, anc_t[i]);
    mt += anc_t[i];
    mb += anc_b[i];
  }
  if (tmax - tmin < MIN_TEMP_SPAN) return;
  mt /= anc_n;
  mb /= anc_n;
  float sxx = 0, sxy = 0;
  for (int i = 0; i < anc_n; ++i) {
    sxx += (anc_t[i] - mt) * (anc_t[i] - mt);
    sxy += (anc_t[i] - mt) * (anc_b[i] - mb);
  }
  const float kmax = deg2rad(0.5f);
  k_temp = clampf(sxy / sxx, -kmax, kmax);
  has_k = true;
}

void HeadingEstimator::update(float gz, float dl, float dr, bool straight, bool still, const Config& c, float dt,
                              bool calibrating, float temp_c) {
  temp = temp_c;
  still = still && std::fabs(gz - bias) < deg2rad(c.imu_still_gyro_dps);
  if (still) {
    still_t += dt;
  } else {
    if (zupt_active && still_t > ANCHOR_MIN_STILL && c.imu_zupt_enable) anchorBias();
    still_t = 0;
    anchor_t = 0;
  }
  zupt_active = still && still_t > ZUPT_DELAY;
  if (zupt_active && c.imu_zupt_enable && !calibrating) {
    bias += c.imu_zupt_gain * (gz - bias);
    anchor_t += dt;
    if (still_t > ANCHOR_MIN_STILL && anchor_t >= ANCHOR_PERIOD) {
      anchor_t = 0;
      anchorBias();
    }
  } else if (!calibrating && c.imu_temp_comp && has_ref) {
    bias = ref_b + k(c) * (temp - ref_t);  // đang chạy: bias theo nhiệt độ hiện tại
  }
  w_gyro = (gz - bias) * c.imu_gyro_scale_z;
  w_enc = (dr - dl) / c.wheel_track_eff_mm / dt;
  if ((zupt_active && c.imu_zupt_enable) || calibrating) {
    w = 0;  // đứng yên: không tích phân góc
  } else {
    const float beta = straight ? c.imu_beta_straight : 0.f;
    w = (1.f - beta) * w_gyro + beta * w_enc;
  }
  psi = wrap(psi + w * dt);
}

void Odometry::update(float dl, float dr, float psi) {
  const float ds = 0.5f * (dl + dr);
  arc_l += dl;
  arc_r += dr;
  const float p0 = has_prev ? psi_prev : psi;
  const float mid = p0 + 0.5f * wrap(psi - p0);
  x += ds * std::cos(mid);
  y += ds * std::sin(mid);
  s += ds;
  psi_prev = psi;
  has_prev = true;
}

}  // namespace tr

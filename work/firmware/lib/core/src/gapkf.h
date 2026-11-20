#pragma once
// Kalman filter 2 trạng thái cho gap: x = [g, v_L] (algorithms §4.2). Tương ứng fw/gapkf.py.

namespace tr {

struct GapKF {
  float g = 0, vL = 0;
  float P[2][2] = {{1e4f, 0}, {0, 1e4f}};
  int rejects = 0;
  bool active = false;

  void reset(float g0, float vL0 = 0, float sigma_g = 10, float sigma_v = 150) {
    g = g0;
    vL = vL0;
    P[0][0] = sigma_g * sigma_g; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = sigma_v * sigma_v;
    rejects = 0;
    active = true;
  }

  void predict(float vF, float dt, float q_a) {
    g += (vL - vF) * dt;
    const float p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1];
    const float p01 = P[0][1] + dt * P[1][1];
    const float p10 = P[1][0] + dt * P[1][1];
    const float p11 = P[1][1];
    const float q = q_a * q_a;
    P[0][0] = p00 + q * dt * dt * dt * dt / 4;
    P[0][1] = p01 + q * dt * dt * dt / 2;
    P[1][0] = p10 + q * dt * dt * dt / 2;
    P[1][1] = p11 + q * dt * dt;
  }

  // Đo gap trực tiếp (ToF). false nếu bị gating loại.
  bool updateG(float z, float sigma, float gate_chi2) {
    const float s = P[0][0] + sigma * sigma;
    const float nu = z - g;
    if (nu * nu / s > gate_chi2) { ++rejects; return false; }
    rejects = 0;
    const float k0 = P[0][0] / s, k1 = P[1][0] / s;
    g += k0 * nu;
    vL += k1 * nu;
    const float p00 = P[0][0], p01 = P[0][1], p10 = P[1][0], p11 = P[1][1];
    P[0][0] = (1 - k0) * p00; P[0][1] = (1 - k0) * p01;
    P[1][0] = p10 - k1 * p00; P[1][1] = p11 - k1 * p01;
    return true;
  }

  // Đo vận tốc leader (relay qua PC)
  void updateV(float z, float sigma) {
    const float s = P[1][1] + sigma * sigma;
    const float nu = z - vL;
    const float k0 = P[0][1] / s, k1 = P[1][1] / s;
    g += k0 * nu;
    vL += k1 * nu;
    const float p00 = P[0][0], p01 = P[0][1], p10 = P[1][0], p11 = P[1][1];
    P[0][0] = p00 - k0 * p10; P[0][1] = p01 - k0 * p11;
    P[1][0] = (1 - k1) * p10; P[1][1] = (1 - k1) * p11;
  }
};

}  // namespace tr

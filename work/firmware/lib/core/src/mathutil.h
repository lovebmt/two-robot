#pragma once
#include <cmath>

namespace tr {

constexpr float PI_F = 3.14159265358979f;

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float sgn(float x) { return x > 0 ? 1.f : (x < 0 ? -1.f : 0.f); }
inline float deg2rad(float d) { return d * PI_F / 180.f; }
inline float rad2deg(float r) { return r * 180.f / PI_F; }

// Đưa góc về [-pi, pi)
inline float wrap(float a) {
  a = std::fmod(a + PI_F, 2.f * PI_F);
  if (a < 0) a += 2.f * PI_F;
  return a - PI_F;
}

}  // namespace tr

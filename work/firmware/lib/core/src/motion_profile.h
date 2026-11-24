#pragma once
// Giới hạn đạo hàm gia tốc (jerk) cho vận tốc và vận tốc góc.

#include <cmath>

#include "mathutil.h"

namespace tr {

class JerkRamp {
 public:
  void reset(float v0 = 0) {
    v_ = v0;
    a_ = 0;
  }

  float acceleration() const { return a_; }
  float velocity() const { return v_; }

  float step(float target, float accel_limit, float dt) {
    if (dt <= 0 || accel_limit <= 0) return v_;
    const float desired = clampf((target - v_) / dt, -accel_limit, accel_limit);
    const float jerk = 4.f * accel_limit;
    a_ = clampf(desired, a_ - jerk * dt, a_ + jerk * dt);
    const float previous = v_;
    v_ += a_ * dt;
    if (target != previous && (target - previous) * (target - v_) <= 0) {
      v_ = target;
      a_ = 0;
    }
    return v_;
  }

 private:
  float v_ = 0, a_ = 0;
};

}  // namespace tr

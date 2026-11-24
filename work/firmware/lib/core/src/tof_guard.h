#pragma once
// Phát hiện một hoặc hai mắt ToF bất thường khi follower đang FOLLOW.

#include <algorithm>
#include <cmath>

#include "config_gen.h"

namespace tr {

class TofAnomalyGuard {
 public:
  void reset() {
    nl_ = nr_ = 0;
    il_ = ir_ = 0;
    streak_l_ = streak_r_ = 0;
    armed_ = false;
  }

  bool update(float l_mm, int l_st, float r_mm, int r_st, const Config& c) {
    streak_l_ = side_(l_mm, l_st, hl_, nl_, il_, streak_l_, c);
    streak_r_ = side_(r_mm, r_st, hr_, nr_, ir_, streak_r_, c);
    if (!armed_) {
      armed_ = nl_ >= 3 && nr_ >= 3;
      return false;
    }
    if (streak_l_ >= c.safety_tof_anomaly_samples || streak_r_ >= c.safety_tof_anomaly_samples) {
      reset();
      return true;
    }
    return false;
  }

 private:
  float hl_[5] = {0}, hr_[5] = {0};
  int nl_ = 0, nr_ = 0, il_ = 0, ir_ = 0;
  int streak_l_ = 0, streak_r_ = 0;
  bool armed_ = false;

  static bool valid_(float mm, int st, const Config& c) {
    return st == 0 && mm >= c.range_min_valid_mm && mm <= c.range_max_valid_mm;
  }

  static float median_(const float* h, int n) {
    float tmp[5];
    std::copy(h, h + n, tmp);
    std::sort(tmp, tmp + n);
    return tmp[n / 2];
  }

  static int side_(float mm, int st, float* hist, int& n, int& i, int streak, const Config& c) {
    if (!valid_(mm, st, c)) return n > 0 ? streak + 1 : 0;
    if (n >= 3 && median_(hist, n) - mm >= c.safety_tof_anomaly_jump_mm) ++streak;
    else streak = 0;
    hist[i] = mm;
    i = (i + 1) % 5;
    if (n < 5) ++n;
    return streak;
  }
};

}  // namespace tr

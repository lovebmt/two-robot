#pragma once
// 2 ToF → gap tâm–tâm ρ, góc lệch Δψ (algorithms §4.1). Tương ứng fw/rangeproc.py.
// Quy ước: dpsi = yaw_leader − yaw_follower = atan2(d_r − d_l, b); mắt trái ở y = +b/2.
#include <stdint.h>

#include "config_gen.h"

namespace tr {

struct RangeMeas {
  bool valid = false;
  float rho = 0;
  bool has_dpsi = false;
  float dpsi = 0;
  bool has_dl = false, has_dr = false;
  float dl = 0, dr = 0;
  int edge = 0;  // +1: chỉ mắt trái thấy leader; -1: chỉ mắt phải
  uint32_t t_ms = 0;
};

class RangeProcessor {
 public:
  void reset() { nl_ = nr_ = 0; }
  RangeMeas process(float l_mm, int l_st, float r_mm, int r_st, const Config& c, float leader_rear);

 private:
  float hl_[3] = {0}, hr_[3] = {0};
  int nl_ = 0, nr_ = 0, il_ = 0, ir_ = 0;
  static bool side_(float mm, int st, float off, const Config& c, float* hist, int& n, int& i, float& out);
};

}  // namespace tr

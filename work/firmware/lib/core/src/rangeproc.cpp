#include "rangeproc.h"

#include <algorithm>
#include <cmath>

namespace tr {

bool RangeProcessor::side_(float mm, int st, float off, const Config& c, float* hist, int& n, int& i, float& out) {
  if (st != 0 || mm < c.range_min_valid_mm || mm > c.range_max_valid_mm) {
    n = 0;
    i = 0;
    return false;
  }
  hist[i] = mm + off;
  i = (i + 1) % 3;
  if (n < 3) ++n;
  float tmp[3];
  for (int k = 0; k < n; ++k) tmp[k] = hist[k];
  std::sort(tmp, tmp + n);
  out = tmp[n / 2];  // trung vị
  return true;
}

RangeMeas RangeProcessor::process(float l_mm, int l_st, float r_mm, int r_st, const Config& c, float leader_rear) {
  RangeMeas m;
  float dl = 0, dr = 0;
  m.has_dl = side_(l_mm, l_st, c.range_offset_l_mm, c, hl_, nl_, il_, dl);
  m.has_dr = side_(r_mm, r_st, c.range_offset_r_mm, c, hr_, nr_, ir_, dr);
  m.dl = dl;
  m.dr = dr;
  const float xs = c.range_x_mm, b = c.range_baseline_mm;
  if (m.has_dl && m.has_dr && std::fabs(dl - dr) <= c.range_max_diff_mm) {
    m.valid = true;
    m.has_dpsi = true;
    m.dpsi = std::atan2(dr - dl, b);
    m.rho = xs + 0.5f * (dl + dr) * std::cos(m.dpsi) + leader_rear;
  } else if (m.has_dl && (!m.has_dr || dl < dr)) {
    m.valid = true;
    m.rho = xs + dl + leader_rear;
    m.edge = +1;
  } else if (m.has_dr) {
    m.valid = true;
    m.rho = xs + dr + leader_rear;
    m.edge = -1;
  }
  return m;
}

}  // namespace tr

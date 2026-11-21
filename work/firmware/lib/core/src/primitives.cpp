#include "primitives.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>

#include "app.h"
#include "mathutil.h"

namespace tr {

static float headingHold(App& a, float target, float& integ, float dt) {
  const Config& c = a.params.active();
  const float e = wrap(target - a.heading.psi);
  integ = clampf(integ + c.pid_heading_ki * e * dt, -0.3f, 0.3f);
  return c.pid_heading_kp * e + integ;
}

// ---------------------------------------------------------------- DRIVE_TO
class DriveTo : public Prim {
 public:
  DriveTo() { name = "DRIVE_TO"; }
  float s_end = 0, v_max = 0, hold = 0, hint = 0, v_prev = 0, stall = 0;
  void start(App& a) override {
    const Config& c = a.params.active();
    s_end = argf("s_end", a.odo.s);
    v_max = argf("v_max", 0) > 0 ? argf("v_max", 0) : c.motion_v_cruise;
    hold = has("heading_deg") ? deg2rad(argf("heading_deg", 0)) : a.heading.psi;
    v_prev = std::max(0.f, a.drive.vMeas());
  }
  Cmd update(App& a, float dt) override {
    const Config& c = a.params.active();
    const float rem = s_end - a.odo.s;
    // Kẹt sát đích (ma sát tĩnh) quá stall_time → coi như tới, không chờ mãi
    stall = (rem <= c.motion_settle_tol_mm && std::fabs(a.drive.vMeas()) < 2.f) ? stall + dt : 0.f;
    if (rem <= c.motion_stop_tol_mm || stall > c.motion_stall_time_s) {
      result["s"] = a.odo.s;
      result["yaw_deg"] = rad2deg(a.heading.psi);
      result["overshoot_mm"] = -rem;
      if (rem > c.motion_stop_tol_mm) result["stalled"] = true;
      finish();
      return BRAKE;
    }
    const float acc = c.motion_a_max, vc = c.motion_v_corner, app = c.motion_approach_dist;
    float v = rem > app ? std::min(v_max, std::sqrt(vc * vc + 2 * acc * (rem - app)))
                        : std::max(std::min(vc, c.pid_pos_kp * rem), c.motor_v_min_reliable);
    v = std::min(v, v_prev + acc * dt);
    v_prev = v;
    return {v, headingHold(a, hold, hint, dt), false};
  }
};

// ---------------------------------------------------------------- TURN
// ω tối thiểu khi quay tại chỗ: bánh không được chậm hơn motor.v_min_reliable (dưới mức đó motor không kéo nổi)
static float turnWMin(const Config& c) {
  return std::max(deg2rad(6.f), c.motor_v_min_reliable / std::max(c.wheel_track_eff_mm / 2.f, 1.f));
}

class Turn : public Prim {
 public:
  Turn() { name = "TURN"; }
  float target = 0, w_prev = 0, settle = 0, stall = 0;
  void start(App& a) override {
    const float tgt = has("heading_deg") ? deg2rad(argf("heading_deg", 0)) : a.heading.psi + deg2rad(argf("deg", 0));
    target = wrap(tgt);
  }
  Cmd update(App& a, float dt) override {
    const Config& c = a.params.active();
    const float e = wrap(target - a.heading.psi);
    stall = (std::fabs(e) < deg2rad(c.motion_settle_tol_deg) && std::fabs(a.heading.w) < deg2rad(2.f)) ? stall + dt
                                                                                                      : 0.f;
    if (std::fabs(e) < deg2rad(c.motion_turn_tol_deg) || stall > c.motion_stall_time_s) {
      settle += dt;
      if (settle > 0.15f || stall > c.motion_stall_time_s) {
        result["yaw_deg"] = rad2deg(a.heading.psi);
        result["err_deg"] = rad2deg(e);
        if (std::fabs(e) >= deg2rad(c.motion_turn_tol_deg)) result["stalled"] = true;
        finish();
      }
      return BRAKE;
    }
    settle = 0;
    const float wmax = deg2rad(c.motion_turn_rate_dps), alpha = deg2rad(c.motion_turn_acc_dps2);
    float w = std::min(wmax, 0.8f * std::sqrt(2 * alpha * std::fabs(e)));
    w = std::max(w, turnWMin(c));
    w = std::min(w, std::fabs(w_prev) + alpha * dt);
    w_prev = w;
    return {0, sgn(e) * w, false};
  }
};

static std::unique_ptr<Turn> makeTurn(App& a, const char* key, float val) {
  std::unique_ptr<Turn> t(new Turn());
  t->id = -1;
  t->args[key] = val;
  t->start(a);
  return t;
}

// ---------------------------------------------------------------- HOLD
class Hold : public Prim {
 public:
  Hold() { name = "HOLD"; }
  void start(App&) override { finish(); }
};

// ---------------------------------------------------------------- FOLLOW
class Follow : public Prim {
 public:
  Follow() { name = "FOLLOW"; }
  float g_t = 0, g_min = 0, v_max = 0, integ = 0, v_prev = 0, dpsi_f = 0, hold = 0, hint = 0;
  bool has_limit = false, have_dpsi = false;
  float s_limit = 0;
  int edge = 0;
  uint32_t last_valid = 0, last_dpsi = 0;
  float gTarget() const { return g_t; }
  void start(App& a) override {
    const Config& c = a.params.active();
    g_t = argf("g_target", 0) > 0 ? argf("g_target", 0) : c.gap_target_mm;
    g_min = argf("gap_min_safe", 0);
    has_limit = has("s_limit");
    s_limit = argf("s_limit", 0);
    v_max = argf("v_max", 0) > 0 ? argf("v_max", 0) : c.motion_v_cruise * c.follow_v_headroom;
    const uint32_t now = a.hal.nowMs();
    const bool fresh = a.has_meas && a.meas.valid && now - a.meas.t_ms < 200;
    a.kf.reset(fresh ? a.meas.rho : g_t, 0, c.range_sigma_mm);
    v_prev = std::max(0.f, a.drive.vMeas());
    hold = a.heading.psi;
    last_valid = now;
    last_dpsi = now > 10000 ? now - 10000 : 0;
  }
  void onRange(App& a, const RangeMeas& m) override {
    if (!m.valid) return;
    const Config& c = a.params.active();
    bool ok = a.kf.updateG(m.rho, c.range_sigma_mm, c.follow_kf_gate_chi2);
    if (!ok && a.kf.rejects >= 8) {  // KF lệch hẳn → khởi tạo lại theo số đo
      a.kf.reset(m.rho, a.kf.vL, c.range_sigma_mm);
      ok = true;
    }
    if (!ok) return;
    last_valid = m.t_ms;
    if (m.has_dpsi) {
      dpsi_f = have_dpsi ? dpsi_f + 0.4f * (m.dpsi - dpsi_f) : m.dpsi;
      have_dpsi = true;
      last_dpsi = m.t_ms;
      edge = 0;
    } else {
      edge = m.edge;
    }
  }
  Cmd update(App& a, float dt) override {
    const Config& c = a.params.active();
    const uint32_t now = a.hal.nowMs();
    if (now - last_valid > static_cast<uint32_t>(c.follow_lost_timeout_ms)) {
      a.raiseFault("LEADER_LOST");
      return BRAKE;
    }
    a.kf.predict(a.drive.vMeas(), dt, c.follow_kf_q_a);
    if (a.relay_valid && now - a.relay_t < 300) a.kf.updateV(a.relay_v, 40.f);
    const float e = a.kf.g - g_t;
    integ = clampf(integ + c.follow_ki_g * e * dt, -60.f, 60.f);
    float v = a.kf.vL + c.follow_kp_g * e + integ;
    const float acc = c.motion_a_max;
    v = clampf(v, 0, v_max);
    v = std::min(v, v_prev + acc * dt);
    if (has_limit) {
      const float rem = s_limit - a.odo.s;
      if (rem <= c.motion_stop_tol_mm) {
        result["s"] = a.odo.s;
        result["gap"] = a.kf.g;
        finish();
        return BRAKE;
      }
      v = std::min(v, std::sqrt(2 * acc * rem));
    }
    if (a.kf.g < g_min) {
      v = 0;
      integ = std::min(integ, 0.f);
    }
    v_prev = v;
    // Hướng: song song leader theo Δψ (ToF); mất Δψ thì giữ hướng bằng gyro
    dpsi_f -= a.heading.w * dt;
    float w;
    if (have_dpsi && now - last_dpsi < 250) {
      w = c.follow_k_psi * dpsi_f;
      hold = a.heading.psi;
    } else {
      w = headingHold(a, hold, hint, dt) + c.follow_k_edge * edge;
    }
    if (v < 1.f) return BRAKE;
    return {v, w, false};
  }
};

// ---------------------------------------------------------------- SNAPSHOT
class Snapshot : public Prim {
 public:
  Snapshot() { name = "SNAPSHOT"; standalone = true; needs_imu = false; }
  int n = 12, seen_l = 0, seen_r = 0;
  uint32_t t0 = 0;
  struct S { float rho, dpsi, dl, dr; };
  std::vector<S> samples;
  void start(App& a) override {
    n = static_cast<int>(argf("n", 0)) > 0 ? static_cast<int>(argf("n", 0)) : a.params.active().corner_snapshot_n;
    t0 = a.hal.nowMs();
  }
  void onRange(App&, const RangeMeas& m) override {
    seen_l += m.has_dl;
    seen_r += m.has_dr;
    if (m.valid && m.has_dpsi) samples.push_back({m.rho, m.dpsi, m.dl, m.dr});
  }
  Cmd update(App& a, float) override {
    if (static_cast<int>(samples.size()) >= n) {
      const float k = static_cast<float>(samples.size());
      float mean = 0, dp = 0, dl = 0, dr = 0;
      for (auto& x : samples) { mean += x.rho; dp += x.dpsi; dl += x.dl; dr += x.dr; }
      mean /= k;
      float var = 0;
      for (auto& x : samples) var += (x.rho - mean) * (x.rho - mean);
      const Config& c = a.params.active();
      result["rho"] = mean;
      result["std"] = std::sqrt(var / k);
      result["dpsi_deg"] = rad2deg(dp / k);
      result["dl_mm"] = dl / k;
      result["dr_mm"] = dr / k;
      result["offset_l_mm"] = c.range_offset_l_mm;
      result["offset_r_mm"] = c.range_offset_r_mm;
      result["s"] = a.odo.s;
      result["n"] = static_cast<int>(k);
      finish();
    } else if (a.hal.nowMs() - t0 > 3000) {
      result["n"] = static_cast<int>(samples.size());
      result["seen_l"] = seen_l;
      result["seen_r"] = seen_r;
      finish(false, !(seen_l || seen_r) ? "NO_TARGET" : !(seen_l && seen_r) ? "ONE_EYE_ONLY" : "EYES_DISAGREE");
    }
    return BRAKE;
  }
};

// ---------------------------------------------------------------- ALIGN
class Align : public Prim {
 public:
  Align() { name = "ALIGN"; }
  static constexpr int MAX_ITER = 4;
  float tol = 0;
  int iter = 0, need = 12;
  bool measuring = true;
  uint32_t t0 = 0;
  std::vector<float> dps;
  std::unique_ptr<Turn> turn;
  void measure_(App& a) {
    measuring = true;
    dps.clear();
    need = a.params.active().corner_snapshot_n;
    t0 = a.hal.nowMs();
  }
  void start(App& a) override {
    tol = deg2rad(argf("tol_deg", 0) > 0 ? argf("tol_deg", 0) : a.params.active().corner_align_tol_deg);
    measure_(a);
  }
  void onRange(App&, const RangeMeas& m) override {
    if (measuring && m.valid && m.has_dpsi) dps.push_back(m.dpsi);
  }
  Cmd update(App& a, float dt) override {
    if (measuring) {
      if (static_cast<int>(dps.size()) >= need) {
        float d = 0;
        for (float x : dps) d += x;
        d /= dps.size();
        if (std::fabs(d) < tol || iter >= MAX_ITER) {
          result["dpsi_deg"] = rad2deg(d);
          result["iters"] = iter;
          finish();
          return BRAKE;
        }
        ++iter;
        turn = makeTurn(a, "deg", rad2deg(d));
        measuring = false;
      } else if (a.hal.nowMs() - t0 > 2000) {
        finish(false, "NO_TARGET");
      }
      return BRAKE;
    }
    Cmd out = turn->update(a, dt);
    if (turn->done) measure_(a);
    return out;
  }
};

// ---------------------------------------------------------------- REACQUIRE
class Reacquire : public Prim {
 public:
  Reacquire() { name = "REACQUIRE"; }
  float rho_exp = 0, tol = 0, base = 0;
  std::vector<float> offsets;
  size_t idx = 0;
  bool measuring = true;
  uint32_t t0 = 0;
  std::vector<float> rhos;
  std::unique_ptr<Turn> turn;
  void beginMeasure_(App& a) { measuring = true; rhos.clear(); t0 = a.hal.nowMs(); }
  void start(App& a) override {
    const Config& c = a.params.active();
    rho_exp = argf("rho_exp", 0);
    tol = argf("tol", 0) > 0 ? argf("tol", 0) : c.corner_reacquire_tol_mm;
    const float scan = has("scan_deg") ? argf("scan_deg", 0) : c.corner_scan_deg;
    base = a.heading.psi;
    offsets = {0.f};
    for (float k = 3.f; k <= scan + 1e-6f; k += 3.f) { offsets.push_back(-k); offsets.push_back(k); }
    beginMeasure_(a);
  }
  void next_(App& a) {
    if (++idx >= offsets.size()) { finish(false, "LEADER_NOT_FOUND"); return; }
    turn = makeTurn(a, "heading_deg", rad2deg(base) + offsets[idx]);
    measuring = false;
  }
  void onRange(App&, const RangeMeas& m) override {
    if (measuring && m.valid) rhos.push_back(m.rho);
  }
  Cmd update(App& a, float dt) override {
    if (!measuring) {
      Cmd out = turn->update(a, dt);
      if (turn->done) beginMeasure_(a);
      return out;
    }
    if (rhos.size() >= 4) {
      float r = 0;
      for (float x : rhos) r += x;
      r /= rhos.size();
      if (std::fabs(r - rho_exp) < tol) {
        result["rho"] = r;
        result["off_deg"] = offsets[idx];
        finish();
      } else {
        next_(a);
      }
    } else if (a.hal.nowMs() - t0 > 800) {
      next_(a);
    }
    return BRAKE;
  }
};

// ---------------------------------------------------------------- TOF_CAL
bool tofFit(const std::vector<TofSample>& samples, JsonObject out) {
  if (samples.size() < 40) { out["err"] = "TOO_FEW_SAMPLES"; out["n"] = samples.size(); return false; }
  // Gom theo ô 1°, trung vị d̄ mỗi ô
  std::map<int, std::vector<float>> bins;
  for (auto& s : samples) bins[static_cast<int>(std::floor(rad2deg(s.th)))].push_back(0.5f * (s.dl + s.dr));
  std::vector<std::pair<float, float>> prof;
  for (auto& kv : bins) {
    auto v = kv.second;
    std::sort(v.begin(), v.end());
    prof.push_back({kv.first + 0.5f, v[v.size() / 2]});
  }
  size_t imin = 0;
  for (size_t i = 1; i < prof.size(); ++i)
    if (prof[i].second < prof[imin].second) imin = i;
  const float dmin = prof[imin].second;
  const float thr = dmin + std::max(10.f, 0.05f * dmin);
  auto cross = [&](int dir, float& outx) -> bool {
    int prev = -1;
    for (int i = static_cast<int>(imin); i >= 0 && i < static_cast<int>(prof.size()); i += dir) {
      if (prof[i].second > thr) {
        if (prev < 0) return false;
        const float a = prof[prev].first, va = prof[prev].second, b = prof[i].first, vb = prof[i].second;
        outx = a + (thr - va) * (b - a) / (vb - va);
        return true;
      }
      prev = i;
    }
    return false;
  };
  float left, right;
  if (!cross(-1, left) || !cross(+1, right)) { out["err"] = "PERP_OUT_OF_SCAN"; out["d_min_mm"] = dmin; return false; }
  const float th0 = deg2rad(0.5f * (left + right));
  const float half = deg2rad(0.5f * std::fabs(right - left));
  const float win = std::max(deg2rad(3.f), std::min(deg2rad(8.f), half - deg2rad(2.f)));
  std::vector<float> xs, ys, ms;
  for (auto& s : samples)
    if (std::fabs(s.th - th0) <= win) { xs.push_back(s.th - th0); ys.push_back(s.dr - s.dl); ms.push_back(0.5f * (s.dl + s.dr)); }
  if (xs.size() < 15) { out["err"] = "TOO_FEW_NEAR"; out["n"] = xs.size(); return false; }
  const float n = static_cast<float>(xs.size());
  float mx = 0, my = 0;
  for (size_t i = 0; i < xs.size(); ++i) { mx += xs[i]; my += ys[i]; }
  mx /= n; my /= n;
  float sxx = 0, sxy = 0;
  for (size_t i = 0; i < xs.size(); ++i) { sxx += (xs[i] - mx) * (xs[i] - mx); sxy += (xs[i] - mx) * (ys[i] - my); }
  const float slope = sxy / sxx, c = my - slope * mx;
  float rr = 0;
  for (size_t i = 0; i < xs.size(); ++i) { const float r = ys[i] - (c + slope * xs[i]); rr += r * r; }
  std::sort(ms.begin(), ms.end());
  out["c_mm"] = c;
  out["baseline_mm"] = -slope;
  out["theta0_deg"] = rad2deg(th0);
  out["plateau_deg"] = rad2deg(2 * half);
  out["d_perp_mm"] = ms[ms.size() / 2];
  out["resid_mm"] = std::sqrt(rr / n);
  out["n"] = samples.size();
  return true;
}

class TofCal : public Prim {
 public:
  TofCal() { name = "TOF_CAL"; standalone = true; }
  float scan = 0, rate = 0, base = 0, off_l0 = 0, off_r0 = 0;
  int phase = 0;  // 0 tới -scan, 1 quét lên, 2 quét về, 3 về giữa
  std::vector<TofSample> samples;
  std::unique_ptr<Turn> turn;
  void start(App& a) override {
    const Config& c = a.params.active();
    scan = deg2rad(argf("scan_deg", 0) > 0 ? argf("scan_deg", 0) : 40.f);
    rate = deg2rad(argf("rate_dps", 0) > 0 ? argf("rate_dps", 0) : 8.f);
    base = a.heading.psi;
    off_l0 = c.range_offset_l_mm;
    off_r0 = c.range_offset_r_mm;
    samples.reserve(800);
    turn = makeTurn(a, "heading_deg", rad2deg(base - scan));
  }
  void onRange(App& a, const RangeMeas&) override {
    if (phase != 1 && phase != 2) return;
    const Config& c = a.params.active();
    if (a.raw_ls == 0 && a.raw_rs == 0 && std::min(a.raw_l, a.raw_r) >= c.range_min_valid_mm &&
        std::max(a.raw_l, a.raw_r) <= c.range_max_valid_mm)
      samples.push_back({wrap(a.heading.psi - base), a.raw_l, a.raw_r});
  }
  Cmd update(App& a, float dt) override {
    if (phase == 0 || phase == 3) {
      Cmd out = turn->update(a, dt);
      if (turn->done) {
        if (phase == 0) phase = 1;
        else finish_(a);
      }
      return out;
    }
    const float rel = wrap(a.heading.psi - base);
    if (phase == 1) {
      if (rel >= scan) { phase = 2; return BRAKE; }
      return {0, rate, false};
    }
    if (rel <= -scan) {
      phase = 3;
      turn = makeTurn(a, "heading_deg", rad2deg(base));
      return BRAKE;
    }
    return {0, -rate, false};
  }
  void finish_(App& a) {
    const Config& c = a.params.active();
    JsonObject r = result.to<JsonObject>();
    if (!tofFit(samples, r)) {
      std::string e = r["err"] | "FIT_FAILED";
      finish(false, e.c_str());
      return;
    }
    // Số đo đã bù = thô + offset. Muốn (r − l) = 0 khi vuông góc: off_r − off_l = −c
    const float cc = r["c_mm"];
    float mean_off = 0;
    if (has("wall_mm")) mean_off = (argf("wall_mm", 0) - c.range_x_mm) - r["d_perp_mm"].as<float>();
    r["offset_l_mm"] = mean_off + cc / 2;
    r["offset_r_mm"] = mean_off - cc / 2;
    r["old_offset_l_mm"] = off_l0;
    r["old_offset_r_mm"] = off_r0;
    r["dpsi_bias_deg"] = rad2deg(std::atan2(cc + off_r0 - off_l0, c.range_baseline_mm));
    finish();
  }
};

// ---------------------------------------------------------------- CAL_DRIVE / CAL_TURN
class CalDrive : public Prim {
 public:
  CalDrive() { name = "CAL_DRIVE"; standalone = true; }
  float s0 = 0, al0 = 0, ar0 = 0, psi0 = 0, mm = 1000;
  std::unique_ptr<DriveTo> drv;
  void start(App& a) override {
    s0 = a.odo.s; al0 = a.odo.arc_l; ar0 = a.odo.arc_r; psi0 = a.heading.psi;
    mm = argf("mm", 1000.f);
    const Config& c = a.params.active();
    drv.reset(new DriveTo());
    drv->args["s_end"] = s0 + mm;
    drv->args["heading_deg"] = rad2deg(psi0);
    drv->args["v_max"] = argf("v_max", 0) > 0 ? argf("v_max", 0) : std::min(c.motion_v_cruise, 150.f);
    drv->start(a);
  }
  Cmd update(App& a, float dt) override {
    Cmd out = drv->update(a, dt);
    if (drv->done) {
      result["cmd_mm"] = mm;
      result["enc_mm"] = a.odo.s - s0;
      result["arc_l_mm"] = a.odo.arc_l - al0;
      result["arc_r_mm"] = a.odo.arc_r - ar0;
      result["yaw_change_deg"] = rad2deg(wrap(a.heading.psi - psi0));
      finish();
    }
    return out;
  }
};

class CalTurn : public Prim {
 public:
  CalTurn() { name = "CAL_TURN"; standalone = true; }
  float target = 0, acc = 0, al0 = 0, ar0 = 0, w_prev = 0, settle = 0;
  void start(App& a) override {
    target = deg2rad(argf("deg", 360.f));
    al0 = a.odo.arc_l; ar0 = a.odo.arc_r;
  }
  Cmd update(App& a, float dt) override {
    const Config& c = a.params.active();
    acc += a.heading.w * dt;  // góc cộng dồn (không wrap)
    const float e = target - acc;
    if (std::fabs(e) < deg2rad(c.motion_turn_tol_deg)) {
      settle += dt;
      if (settle > 0.3f) {
        result["cmd_deg"] = rad2deg(target);
        result["est_deg"] = rad2deg(acc);
        result["arc_l_mm"] = a.odo.arc_l - al0;
        result["arc_r_mm"] = a.odo.arc_r - ar0;
        finish();
      }
      return BRAKE;
    }
    settle = 0;
    const float wmax = deg2rad(c.motion_turn_rate_dps), alpha = deg2rad(c.motion_turn_acc_dps2);
    float w = std::max(std::min(wmax, 0.8f * std::sqrt(2 * alpha * std::fabs(e))), turnWMin(c));
    w = std::min(w, std::fabs(w_prev) + alpha * dt);
    w_prev = w;
    return {0, sgn(e) * w, false};
  }
};

// ---------------------------------------------------------------- CAL_KS
// Đo vùng chết từng bánh: tăng điện áp dần (1.5 V/s) tới khi bánh quay (→ điện áp BỨT ma sát tĩnh), rồi giảm dần
// (3 V/s) tới khi bánh dừng (→ điện áp GIỮ quay = vùng chết động, motor.ks). Làm chiều tiến rồi chiều lùi.
// Hai bánh chạy song song, độc lập; xe đi thẳng-ish vài cm rồi lùi về.
class CalKs : public Prim {
 public:
  CalKs() { name = "CAL_KS"; standalone = true; needs_imu = false; }
  enum Ph { UP, DOWN, DONE };
  Ph ph[2] = {UP, UP};
  float V[2] = {0, 0}, brk[2][2] = {{0, 0}, {0, 0}}, stp[2][2] = {{0, 0}, {0, 0}};
  int dir = 1;
  float rest = 0, t = 0;
  static constexpr float RATE_UP = 1.5f, RATE_DOWN = 3.f, V_MAX = 9.f, MOVE = 15.f, STOP = 4.f;
  Cmd update(App& a, float dt) override {
    t += dt;
    if (t > 30.f) { finish(false, "TIMEOUT"); return BRAKE; }
    const float vm[2] = {a.drive.v_meas_l * dir, a.drive.v_meas_r * dir};
    const int di = dir > 0 ? 0 : 1;
    for (int i = 0; i < 2; ++i) {
      if (ph[i] == UP) {
        V[i] += RATE_UP * dt;
        if (vm[i] > MOVE) { brk[i][di] = V[i]; ph[i] = DOWN; }
        else if (V[i] > V_MAX) { finish(false, i == 0 ? "NO_MOVE_L" : "NO_MOVE_R"); return BRAKE; }
      } else if (ph[i] == DOWN) {
        V[i] -= RATE_DOWN * dt;
        if (vm[i] < STOP || V[i] <= 0) { stp[i][di] = std::max(V[i], 0.f); ph[i] = DONE; }
      }
    }
    if (ph[0] == DONE && ph[1] == DONE) {
      rest += dt;
      if (rest < 0.5f) return BRAKE;
      if (dir > 0) {
        dir = -1; rest = 0; ph[0] = ph[1] = UP; V[0] = V[1] = 0;
        return BRAKE;
      }
      const char* side[2] = {"l", "r"};
      for (int i = 0; i < 2; ++i) {
        result[std::string("break_") + side[i] + "_fwd"] = brk[i][0];
        result[std::string("break_") + side[i] + "_back"] = brk[i][1];
        result[std::string("hold_") + side[i] + "_fwd"] = stp[i][0];
        result[std::string("hold_") + side[i] + "_back"] = stp[i][1];
      }
      result["vbat"] = a.vbat();
      finish();
      return BRAKE;
    }
    Cmd c;
    c.brake = false;
    c.raw = true;
    c.volt_l = ph[0] == DONE ? 0.f : dir * V[0];
    c.volt_r = ph[1] == DONE ? 0.f : dir * V[1];
    return c;
  }
};

std::unique_ptr<Prim> makePrim(const char* n) {
  if (!std::strcmp(n, "CAL_KS")) return std::unique_ptr<Prim>(new CalKs());
  if (!std::strcmp(n, "DRIVE_TO")) return std::unique_ptr<Prim>(new DriveTo());
  if (!std::strcmp(n, "TURN")) return std::unique_ptr<Prim>(new Turn());
  if (!std::strcmp(n, "HOLD")) return std::unique_ptr<Prim>(new Hold());
  if (!std::strcmp(n, "FOLLOW")) return std::unique_ptr<Prim>(new Follow());
  if (!std::strcmp(n, "SNAPSHOT")) return std::unique_ptr<Prim>(new Snapshot());
  if (!std::strcmp(n, "ALIGN")) return std::unique_ptr<Prim>(new Align());
  if (!std::strcmp(n, "REACQUIRE")) return std::unique_ptr<Prim>(new Reacquire());
  if (!std::strcmp(n, "TOF_CAL")) return std::unique_ptr<Prim>(new TofCal());
  if (!std::strcmp(n, "CAL_DRIVE")) return std::unique_ptr<Prim>(new CalDrive());
  if (!std::strcmp(n, "CAL_TURN")) return std::unique_ptr<Prim>(new CalTurn());
  return nullptr;
}

// Follow cần biết g_target cho telemetry
float followTarget(const Prim* p) {
  return p && !std::strcmp(p->name, "FOLLOW") ? static_cast<const Follow*>(p)->gTarget() : -1.f;
}

}  // namespace tr

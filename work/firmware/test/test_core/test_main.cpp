// Test lõi thuật toán trên máy tính: pio test -e native
#include <ArduinoJson.h>
#include <unity.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

#include "app.h"
#include "gapkf.h"
#include "mathutil.h"
#include "motion_profile.h"
#include "params.h"
#include "paramsvc.h"
#include "primitives.h"
#include "rangeproc.h"
#include "tof_guard.h"

using namespace tr;

// HAL giả: thời gian do test điều khiển, gyro có bias cố định, flash trong RAM
struct FakeHal : Hal {
  uint32_t t = 0;
  float gyro_bias = deg2rad(2.0f), temp = 30;
  int32_t enc_l = 0, enc_r = 0;
  float duty_l = 0, duty_r = 0;
  bool brake = true;
  std::string flash, role;
  uint32_t nowMs() override { return t; }
  void setMotor(char s, float d, bool b) override { (s == 'l' ? duty_l : duty_r) = d; brake = b; }
  int32_t readEncoder(char s) override { return s == 'l' ? enc_l : enc_r; }
  float readGyroZ() override { return gyro_bias; }
  float readImuTemp() override { return temp; }
  float readVbat(float) override { return NAN; }
  bool storageLoad(std::string& j) override { if (flash.empty()) return false; j = flash; return true; }
  bool storageSave(const std::string& j) override { flash = j; return true; }
  std::string nvsGetRole() override { return role; }
  void nvsSetRole(const std::string& r) override { role = r; }
};

static void runTicks(App& a, FakeHal& h, float seconds) {
  const int n = static_cast<int>(seconds / 0.01f + 0.5f);
  for (int i = 0; i < n; ++i) { h.t += 10; a.tick(0.01f); }
}

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- params
void test_defaults_valid() {
  Config out;
  Errors e;
  JsonDocument d;
  TEST_ASSERT_TRUE(validatePatch(Config(), d.to<JsonObject>(), out, e));
}

void test_patch_errors() {
  JsonDocument d;
  d["wheel.diameter_mm"] = 5;
  d["body.width_mm"] = "abc";
  d["x.y"] = 1;
  Config out;
  Errors e;
  TEST_ASSERT_FALSE(validatePatch(Config(), d.as<JsonObjectConst>(), out, e));
  TEST_ASSERT_EQUAL(3, e.size());
}

void test_derived() {
  Derived d = derive(Config());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, PI_F * 65.f / (11 * 4 * Config().enc_gear_ratio), d.mm_per_tick_l);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, std::hypot(120.f, 60.f), d.r_sweep_mm);
}

void test_paramservice_swap_idle_lock_and_persist() {
  FakeHal h;
  ParamService ps(h);
  ps.load();
  TEST_ASSERT_TRUE(ps.flag_default);
  JsonDocument p, r;
  p["pid.wheel_kp"] = 0.02;
  ps.submit(p.as<JsonObjectConst>(), true, r.to<JsonObject>());
  TEST_ASSERT_TRUE(r["ok"].as<bool>());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.01f, ps.active().pid_wheel_kp);  // chưa swap
  TEST_ASSERT_TRUE(ps.swapIfPending() & G_PID);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.02f, ps.active().pid_wheel_kp);
  JsonDocument p2, r2;
  p2["wheel.diameter_mm"] = 70;
  ps.submit(p2.as<JsonObjectConst>(), true, r2.to<JsonObject>());
  TEST_ASSERT_FALSE(r2["ok"].as<bool>());
  TEST_ASSERT_TRUE(r2["busy"].as<bool>());
  JsonDocument r3;
  ps.save(r3.to<JsonObject>());
  TEST_ASSERT_FALSE(ps.flag_default);
  ParamService ps2(h);
  ps2.load();
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.02f, ps2.active().pid_wheel_kp);
  TEST_ASSERT_FALSE(ps2.dirty());
}

// ---------------------------------------------------------------- filters
void test_gapkf_tracks_leader_speed() {
  GapKF kf;
  kf.reset(300);
  float g_true = 300;
  for (int i = 0; i < 300; ++i) {  // leader 100 mm/s, follower 80 mm/s
    g_true += (100 - 80) * 0.01f;
    kf.predict(80, 0.01f, 300);
    if (i % 3 == 0) kf.updateG(g_true, 3, 9);
  }
  TEST_ASSERT_FLOAT_WITHIN(5, g_true, kf.g);
  TEST_ASSERT_FLOAT_WITHIN(15, 100, kf.vL);
}

void test_rangeproc_dpsi_sign() {
  RangeProcessor rp;
  Config c;
  RangeMeas m = rp.process(140, 0, 150, 0, c, 40);  // d_l < d_r → leader xoay trái → dpsi > 0
  TEST_ASSERT_TRUE(m.valid && m.has_dpsi);
  TEST_ASSERT_TRUE(m.dpsi > 0);
  TEST_ASSERT_FLOAT_WITHIN(1, 120 + 145 * std::cos(m.dpsi) + 40, m.rho);
  RangeProcessor rp2;
  RangeMeas m2 = rp2.process(140, 0, 8190, 4, c, 40);
  TEST_ASSERT_EQUAL(+1, m2.edge);
}

void test_jerk_ramp_limits_acceleration_changes() {
  JerkRamp ramp;
  const float a_max = 250.f, dt = 0.01f, max_da = 4.f * a_max * dt;
  float previous_a = ramp.acceleration();
  float previous_v = 0.f;
  for (int i = 0; i < 120; ++i) {
    const float v = ramp.step(100.f, a_max, dt);
    const float a = ramp.acceleration();
    if (v < 100.f - 1e-6f) TEST_ASSERT_LESS_OR_EQUAL(max_da + 1e-5f, std::fabs(a - previous_a));
    TEST_ASSERT_TRUE(v >= previous_v);
    previous_v = v;
    previous_a = a;
  }
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 100.f, ramp.velocity());
  previous_a = ramp.acceleration();
  for (int i = 0; i < 120; ++i) {
    const float v = ramp.step(0.f, a_max, dt);
    if (v > 1e-6f) TEST_ASSERT_LESS_OR_EQUAL(max_da + 1e-5f, std::fabs(ramp.acceleration() - previous_a));
    previous_a = ramp.acceleration();
  }
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.f, ramp.velocity());
}

// tofFit trên mẫu tổng hợp: chùm rộng → d̄ phẳng quanh θ0 = 4°, c = −5 mm, b = 60
void test_tof_fit_synthetic() {
  std::vector<TofSample> s;
  const float th0 = deg2rad(4), c = -5, b = 60, D = 200, half = deg2rad(12.5f);
  for (int pass = 0; pass < 2; ++pass)
    for (int i = 0; i <= 600; ++i) {
      const float th = deg2rad(-40.f + 80.f * i / 600.f), phi = th - th0;
      const float extra = std::fabs(phi) > half ? D * (1 / std::cos(std::fabs(phi) - half) - 1) : 0;
      const float base = D - 120 * std::cos(phi) + 120 + extra;
      const float dl = base + 0.5f * b * std::sin(phi) - c / 2, dr = base - 0.5f * b * std::sin(phi) + c / 2;
      s.push_back({th, dl, dr});
    }
  JsonDocument d;
  TEST_ASSERT_TRUE(tofFit(s, d.to<JsonObject>()));
  TEST_ASSERT_FLOAT_WITHIN(0.8, c, d["c_mm"].as<float>());
  TEST_ASSERT_FLOAT_WITHIN(1.0, 4.0, d["theta0_deg"].as<float>());
  TEST_ASSERT_FLOAT_WITHIN(6, b, d["baseline_mm"].as<float>());
}

// Đối chiếu với Python: mẫu ToF thật từ mô phỏng (test/vectors/tof_fit.json do tools/gen_vectors.py sinh)
void test_tof_fit_matches_python() {
  std::ifstream f("test/vectors/tof_fit.json");
  if (!f) { TEST_IGNORE_MESSAGE("thiếu test/vectors/tof_fit.json — chạy tools/gen_vectors.py"); return; }
  std::stringstream ss;
  ss << f.rdbuf();
  JsonDocument v;
  TEST_ASSERT_FALSE(deserializeJson(v, ss.str()));
  for (JsonObjectConst cs : v["cases"].as<JsonArrayConst>()) {
    std::vector<TofSample> s;
    for (JsonArrayConst x : cs["samples"].as<JsonArrayConst>()) s.push_back({x[0], x[1], x[2]});
    JsonDocument d;
    TEST_ASSERT_TRUE(tofFit(s, d.to<JsonObject>()));
    TEST_ASSERT_FLOAT_WITHIN(0.05, cs["expect"]["c_mm"].as<float>(), d["c_mm"].as<float>());
    TEST_ASSERT_FLOAT_WITHIN(0.05, cs["expect"]["theta0_deg"].as<float>(), d["theta0_deg"].as<float>());
    TEST_ASSERT_FLOAT_WITHIN(0.1, cs["expect"]["baseline_mm"].as<float>(), d["baseline_mm"].as<float>());
  }
}

// ---------------------------------------------------------------- App
void test_app_imu_autocalib_and_arm_gate() {
  FakeHal h;
  App a(h, "t1", "test");
  a.begin();
  JsonDocument ok;
  a.params.save(ok.to<JsonObject>());  // xác nhận config (bỏ CFG_DEFAULT)
  JsonDocument body, out;
  body["role"] = "leader";
  TEST_ASSERT_EQUAL(200, a.handleRequest("PUT", "/api/role", body.as<JsonObjectConst>(), out));
  JsonDocument path, out2;
  path["path_id"] = 1;
  path["start"]["x"] = 500; path["start"]["y"] = 300; path["start"]["yaw_deg"] = 90;
  path["formation"]["leader_rear_mm"] = 40;
  TEST_ASSERT_EQUAL(200, a.handleRequest("PUT", "/api/path", path.as<JsonObjectConst>(), out2));
  runTicks(a, h, 0.3f);
  JsonDocument c1, r1;
  c1["op"] = "arm";
  a.handleCmd(c1.as<JsonObjectConst>(), r1.to<JsonObject>());
  TEST_ASSERT_FALSE(r1["ok"].as<bool>());  // đang calib / chưa calib
  TEST_ASSERT_FLOAT_WITHIN(1e-4, deg2rad(90), a.heading.psi);  // chưa calib: không tích phân bias 2°/s
  runTicks(a, h, 3.0f);
  JsonDocument r2;
  a.handleCmd(c1.as<JsonObjectConst>(), r2.to<JsonObject>());
  TEST_ASSERT_TRUE(r2["ok"].as<bool>());
  TEST_ASSERT_FLOAT_WITHIN(0.02, 2.0, rad2deg(a.heading.bias));
  runTicks(a, h, 10.0f);  // đã calib + ZUPT: hướng không trôi
  TEST_ASSERT_FLOAT_WITHIN(deg2rad(0.2f), deg2rad(90), a.heading.psi);
}

void test_app_state_events_and_link_loss() {
  FakeHal h;
  App a(h, "t2", "test");
  a.begin();
  JsonDocument ok;
  a.params.save(ok.to<JsonObject>());
  h.role = "leader";
  a.role = "leader";
  JsonDocument path, o;
  path["start"]["x"] = 0; path["start"]["y"] = 0; path["start"]["yaw_deg"] = 0;
  a.handleRequest("PUT", "/api/path", path.as<JsonObjectConst>(), o);
  runTicks(a, h, 3.0f);  // auto calib
  JsonDocument q, raw;
  q["since"] = 0;
  TEST_ASSERT_EQUAL(200, a.handleRequest("GET", "/api/state", q.as<JsonObjectConst>(), raw));
  std::string wire;
  serializeJson(raw, wire);  // sự kiện nhúng dạng JSON thô → parse lại như PC nhận qua HTTP
  JsonDocument st;
  TEST_ASSERT_FALSE(deserializeJson(st, wire));
  TEST_ASSERT_TRUE(st["events"].size() >= 1);  // calib_done
  TEST_ASSERT_EQUAL_STRING("calib_done", st["events"][0]["e"]);
  TEST_ASSERT_EQUAL(1, st["events"][0]["eseq"].as<int>());
  JsonDocument c, r;
  c["op"] = "arm";
  a.handleCmd(c.as<JsonObjectConst>(), r.to<JsonObject>());
  TEST_ASSERT_TRUE(r["ok"].as<bool>());
  // Heartbeat từ task HTTP mới hơn `now` của chu kỳ (race thật trên ESP32) → KHÔNG được báo LINK_LOST
  for (int i = 0; i < 50; ++i) { a.touchLink(h.t + 10 + 5); h.t += 10; a.tick(0.01f); }
  TEST_ASSERT_EQUAL_STRING("ARMED", a.state.c_str());
  runTicks(a, h, 2.0f);  // không có request nào > link_timeout (1500 ms) → LINK_LOST
  TEST_ASSERT_EQUAL_STRING("FAULT", a.state.c_str());
  TEST_ASSERT_EQUAL_STRING("LINK_LOST", a.fault.c_str());
}

void test_app_unknown_route_and_config() {
  FakeHal h;
  App a(h, "t3", "test");
  a.begin();
  JsonDocument b, o;
  TEST_ASSERT_EQUAL(404, a.handleRequest("GET", "/api/nope", b.as<JsonObjectConst>(), o));
  JsonDocument put, o2;
  put["patch"]["gap.target_mm"] = 400;
  put["persist"] = true;
  TEST_ASSERT_EQUAL(200, a.handleRequest("PUT", "/api/config", put.as<JsonObjectConst>(), o2));
  runTicks(a, h, 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 400, a.params.active().gap_target_mm);
  TEST_ASSERT_TRUE(h.flash.find("\"gap.target_mm\":400") != std::string::npos);
}

void test_app_tof_guard_faults_only_during_follow() {
  FakeHal h;
  App a(h, "guard", "guard");
  a.begin();
  JsonDocument saved;
  a.params.save(saved.to<JsonObject>());
  a.role = h.role = "follower";
  JsonDocument path, out;
  path["path_id"] = 20;
  path["start"]["x"] = 500; path["start"]["y"] = 300; path["start"]["yaw_deg"] = 0;
  path["follower"]["start"]["x"] = 200; path["follower"]["start"]["y"] = 300;
  path["follower"]["start"]["yaw_deg"] = 0; path["follower"]["start_s"] = -300;
  path["formation"]["leader_rear_mm"] = 40;
  TEST_ASSERT_EQUAL(200, a.handleRequest("PUT", "/api/path", path.as<JsonObjectConst>(), out));
  runTicks(a, h, 3.0f);
  JsonDocument arm, armOut;
  arm["op"] = "arm";
  a.handleCmd(arm.as<JsonObjectConst>(), armOut.to<JsonObject>());
  TEST_ASSERT_EQUAL_STRING("ARMED", a.state.c_str());
  JsonDocument prim, primOut;
  prim["op"] = "prim"; prim["id"] = 5; prim["prim"] = "FOLLOW";
  prim["g_target"] = 320; prim["gap_min_safe"] = 175;
  a.handleCmd(prim.as<JsonObjectConst>(), primOut.to<JsonObject>());
  TEST_ASSERT_EQUAL_STRING("RUNNING", a.state.c_str());
  for (int i = 0; i < 3; ++i) a.onRange(300, 0, 302, 0);
  a.onRange(15, 0, 302, 0);
  a.onRange(15, 0, 302, 0);
  TEST_ASSERT_EQUAL_STRING("FAULT", a.state.c_str());
  TEST_ASSERT_EQUAL_STRING("OBSTACLE_INTRUSION", a.fault.c_str());

  FakeHal idleHal;
  App idle(idleHal, "idle", "idle");
  idle.begin();
  for (int i = 0; i < 3; ++i) idle.onRange(300, 0, 302, 0);
  idle.onRange(15, 0, 302, 0);
  idle.onRange(15, 0, 302, 0);
  TEST_ASSERT_EQUAL_STRING("IDLE", idle.state.c_str());
  TEST_ASSERT_TRUE(idle.fault.empty());
}

// Hợp đồng API: mọi khoá JSON firmware Python trả về (server/UI đọc) phải có trong bản C++
static bool hasPath(JsonVariantConst v, const std::string& path) {
  size_t i = 0;
  while (true) {
    const size_t j = path.find('.', i);
    const std::string k = path.substr(i, j == std::string::npos ? std::string::npos : j - i);
    if (!v.is<JsonObjectConst>() || !v.as<JsonObjectConst>()[k.c_str()].is<JsonVariantConst>()) {
      if (!v.is<JsonObjectConst>()) return false;
      JsonObjectConst o = v.as<JsonObjectConst>();
      bool found = false;
      for (JsonPairConst kv : o) if (k == kv.key().c_str()) { found = true; v = kv.value(); break; }
      if (!found) return false;
    } else {
      v = v.as<JsonObjectConst>()[k.c_str()];
    }
    if (j == std::string::npos) return true;
    i = j + 1;
  }
}

void test_api_contract_matches_python() {
  std::ifstream f("test/vectors/api_contract.json");
  if (!f) { TEST_IGNORE_MESSAGE("thiếu test/vectors/api_contract.json — chạy tools/gen_vectors.py"); return; }
  std::stringstream ss;
  ss << f.rdbuf();
  JsonDocument v;
  TEST_ASSERT_FALSE(deserializeJson(v, ss.str()));
  FakeHal h;
  App a(h, "c1", "contract");
  a.begin();
  runTicks(a, h, 3.0f);  // có sự kiện calib_done
  JsonDocument q, raw, info, cfg, e;
  q["since"] = 0;
  a.handleRequest("GET", "/api/state", q.as<JsonObjectConst>(), raw);
  std::string wire;
  serializeJson(raw, wire);
  JsonDocument st;
  deserializeJson(st, wire);
  a.handleRequest("GET", "/api/info", e.as<JsonObjectConst>(), info);
  a.handleRequest("GET", "/api/config", e.as<JsonObjectConst>(), cfg);
  struct { const char* name; JsonVariantConst root; } sets[] = {
      {"state", st.as<JsonVariantConst>()}, {"tel", st["tel"]}, {"info", info.as<JsonVariantConst>()},
      {"config", cfg.as<JsonVariantConst>()}, {"calib_event", st["events"][0]}};
  for (auto& s : sets)
    for (JsonVariantConst k : v[s.name].as<JsonArrayConst>()) {
      const std::string key = k.as<const char*>();
      if (!hasPath(s.root, key)) {
        std::string msg = std::string(s.name) + ": thiếu khoá " + key;
        TEST_FAIL_MESSAGE(msg.c_str());
      }
    }
}


void test_pause_in_idle_does_not_stick() {
  FakeHal h;
  App a(h, "t", "t");
  a.begin();
  JsonDocument c, r;
  c["op"] = "pause";
  a.handleCmd(c.as<JsonObjectConst>(), r.to<JsonObject>());
  TEST_ASSERT_FALSE(a.paused);
  a.paused = true;  // cờ sót từ firmware cũ
  JsonDocument c2, r2;
  c2["op"] = "reset";
  a.handleCmd(c2.as<JsonObjectConst>(), r2.to<JsonObject>());
  TEST_ASSERT_FALSE(a.paused);
}

void test_solo_path_ignores_role() {
  FakeHal h;
  App a(h, "t", "t");
  a.begin();
  a.role = "follower";
  JsonDocument p, out;
  p["solo"] = true;
  p["path_id"] = 7;
  p["start"]["x"] = 120.f; p["start"]["y"] = 40.f; p["start"]["yaw_deg"] = 0.f;
  p["follower"]["start"]["x"] = -300.f; p["follower"]["start"]["y"] = 40.f; p["follower"]["start"]["yaw_deg"] = 0.f;
  p["follower"]["start_s"] = -400.f;
  TEST_ASSERT_EQUAL(200, a.handleRequest("PUT", "/api/path", p.as<JsonObjectConst>(), out));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 120.f, a.odo.x);  // đặt ở điểm đầu như leader
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.f, a.odo.s);
  a.role.clear();
  JsonDocument out2;
  TEST_ASSERT_EQUAL(200, a.handleRequest("PUT", "/api/path", p.as<JsonObjectConst>(), out2));  // không vai trò vẫn được
  p["solo"] = false;
  JsonDocument out3;
  TEST_ASSERT_EQUAL(400, a.handleRequest("PUT", "/api/path", p.as<JsonObjectConst>(), out3));  // 2 xe: cần vai trò
}

static void armTofGuard(TofAnomalyGuard& guard, const Config& c) {
  for (int i = 0; i < 3; ++i) {
    TEST_ASSERT_FALSE(guard.update(300, 0, 302, 0, c));
  }
}

void test_tof_guard_ignores_gradual_change() {
  Config c;
  TofAnomalyGuard g;
  armTofGuard(g, c);
  for (float mm : {280.f, 260.f, 240.f, 220.f, 200.f}) {
    TEST_ASSERT_FALSE(g.update(mm, 0, mm + 2, 0, c));
  }
}

void test_tof_guard_trips_one_or_both_eyes() {
  Config c;
  TofAnomalyGuard one;
  armTofGuard(one, c);
  TEST_ASSERT_FALSE(one.update(170, 0, 302, 0, c));
  TEST_ASSERT_TRUE(one.update(170, 0, 302, 0, c));

  TofAnomalyGuard both;
  armTofGuard(both, c);
  TEST_ASSERT_FALSE(both.update(170, 0, 172, 0, c));
  TEST_ASSERT_TRUE(both.update(170, 0, 172, 0, c));
}

void test_tof_guard_trips_on_signal_loss() {
  Config c;
  TofAnomalyGuard one;
  armTofGuard(one, c);
  TEST_ASSERT_FALSE(one.update(300, 0, 8190, 4, c));
  TEST_ASSERT_TRUE(one.update(300, 0, 8190, 4, c));

  TofAnomalyGuard both;
  armTofGuard(both, c);
  TEST_ASSERT_FALSE(both.update(8190, 4, 8190, 4, c));
  TEST_ASSERT_TRUE(both.update(8190, 4, 8190, 4, c));
}

// Bánh kẹt ma sát tĩnh (lệnh 15 mm/s, đo 0): phải đẩy vượt trần PI cũ (~4.5 V); kbreak cấp ngay từ đầu
static float stallVolts(const Config& c, float seconds) {
  WheelController wc('l');
  float d = 0;
  for (int i = 0; i < static_cast<int>(seconds / 0.01f); ++i) d = wc.update(15.f, 0.f, 12.f, c, 0.01f);
  return d * (12.f - c.power_l298n_vdrop_v);
}

void test_wheel_stall_boost_and_kbreak() {
  Config c;
  TEST_ASSERT_TRUE(stallVolts(c, 0.1f) < 2.f);
  TEST_ASSERT_TRUE(stallVolts(c, 3.f) > 7.5f);
  c.motor_kbreak_l_v = 5.f;
  TEST_ASSERT_TRUE(stallVolts(c, 0.02f) > 5.f);
  WheelController wc('l');
  const float d = wc.update(15.f, 15.f, 12.f, c, 0.01f);  // đang quay → về ks
  TEST_ASSERT_TRUE(d * (12.f - c.power_l298n_vdrop_v) < 2.f);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_defaults_valid);
  RUN_TEST(test_patch_errors);
  RUN_TEST(test_derived);
  RUN_TEST(test_paramservice_swap_idle_lock_and_persist);
  RUN_TEST(test_gapkf_tracks_leader_speed);
  RUN_TEST(test_rangeproc_dpsi_sign);
  RUN_TEST(test_jerk_ramp_limits_acceleration_changes);
  RUN_TEST(test_tof_fit_synthetic);
  RUN_TEST(test_tof_fit_matches_python);
  RUN_TEST(test_app_imu_autocalib_and_arm_gate);
  RUN_TEST(test_app_state_events_and_link_loss);
  RUN_TEST(test_app_unknown_route_and_config);
  RUN_TEST(test_app_tof_guard_faults_only_during_follow);
  RUN_TEST(test_api_contract_matches_python);
  RUN_TEST(test_wheel_stall_boost_and_kbreak);
  RUN_TEST(test_pause_in_idle_does_not_stick);
  RUN_TEST(test_solo_path_ignores_role);
  RUN_TEST(test_tof_guard_ignores_gradual_change);
  RUN_TEST(test_tof_guard_trips_one_or_both_eyes);
  RUN_TEST(test_tof_guard_trips_on_signal_loss);
  return UNITY_END();
}

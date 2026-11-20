#include "app.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "mathutil.h"

namespace tr {

const char* FW_VERSION = "0.2.2-esp32";

App::App(Hal& h, const std::string& id_, const std::string& name_) : hal(h), params(h), id(id_), name(name_) {}

void App::begin() {
  params.load();
  role = hal.nvsGetRole();
  if (role != "leader" && role != "follower") role.clear();
  pwm_hz_ = params.active().motor_pwm_hz;
  hal.setPwmFreq(pwm_hz_);
}

// ================================================================ control (100 Hz)
void App::tick(float dt) {
  const uint32_t groups = params.swapIfPending();
  if (groups) onParams_(groups);
  const Config& c = params.active();
  const Derived& der = params.derived();
  const uint32_t now = hal.nowMs();

  // Cờ đảo chiều (lắp dây ngược) áp dụng ở ranh giới HAL
  const int32_t tl = hal.readEncoder('l') * (c.enc_invert_l ? -1 : 1);
  const int32_t tr = hal.readEncoder('r') * (c.enc_invert_r ? -1 : 1);
  if (!has_last_ticks_) { last_tl_ = tl; last_tr_ = tr; has_last_ticks_ = true; }
  const bool moved = tl != last_tl_ || tr != last_tr_;
  const float dl = (tl - last_tl_) * der.mm_per_tick_l;
  const float dr = (tr - last_tr_) * der.mm_per_tick_r;
  last_tl_ = tl;
  last_tr_ = tr;
  drive.measure(dl, dr, dt);

  const float gz = hal.readGyroZ() * (c.imu_invert_z ? -1.f : 1.f);
  const bool enc_still = drive.braking && !moved;
  enc_still_t_ = enc_still ? enc_still_t_ + dt : 0.f;
  const bool calibrating = imu_cal_ != nullptr;
  if (calibrating) {
    CalibResult r;
    if (imu_cal_->feed(gz, moved, dt, r)) finishCalib_(r);
  } else if (!imu_ok_ && c.imu_auto_calib && state == "IDLE" && enc_still_t_ > 0.5f && !(hal.hwFault() & HW_IMU)) {
    startCalib_();
  }
  const bool still = enc_still && std::fabs(drive.v_meas_l) < 1.f && std::fabs(drive.v_meas_r) < 1.f;
  const bool straight = std::fabs(w_cmd_) < 0.05f && std::fabs(v_cmd_) > 5.f;
  // Chưa calib mà đứng yên: bias chưa biết → không tích phân hướng
  const bool hold = calibrating || (!imu_ok_ && enc_still);
  heading.update(gz, dl, dr, straight, still, c, dt, hold, hal.readImuTemp());
  odo.update(dl, dr, heading.psi);

  const float vb = hal.readVbat(c.power_vbat_divider);
  vbat_ = std::isnan(vb) ? 12.f : vb;
  bat_low_ = !std::isnan(vb) && vb < c.power_bat_min_v;
  if (runState_() && link_watch_ && now - last_hb_ms_ > static_cast<uint32_t>(c.safety_link_timeout_ms))
    raiseFault("LINK_LOST");

  Cmd cmd = BRAKE;
  if (state == "RUNNING" && prim_ && !paused) {
    cmd = prim_->update(*this, dt);
    if (prim_ && prim_->done) finishPrim_();
  } else if (state == "JOG") {
    if (now < jog_until_) cmd = {jog_v_, jog_w_, false};
    else state = "IDLE";
  }
  v_cmd_ = cmd.brake ? 0 : cmd.v;
  w_cmd_ = cmd.brake ? 0 : cmd.w;
  float duty_l, duty_r;
  bool b;
  drive.command(cmd.v, cmd.w, cmd.brake, c, vbat_, dt, duty_l, duty_r, b);
  if (state == "TEST") {  // thử motor thô: 1 bánh duty cố định, bánh kia phanh
    if (now < test_until_) {
      const float d = test_duty_;
      hal.setMotor('l', test_side_ == 'l' ? (c.motor_invert_l ? -d : d) : 0.f, test_side_ != 'l');
      hal.setMotor('r', test_side_ == 'r' ? (c.motor_invert_r ? -d : d) : 0.f, test_side_ != 'r');
      return;
    }
    JsonDocument e;
    e["t"] = "evt"; e["e"] = "motor_test_done"; e["ok"] = true;
    JsonObject r = e["result"].to<JsonObject>();
    r["side"] = test_side_ == 'l' ? "l" : "r";
    r["duty"] = test_duty_;
    r["ticks_l"] = tl - test_tl0_;
    r["ticks_r"] = tr - test_tr0_;
    r["mm_l"] = (tl - test_tl0_) * der.mm_per_tick_l;
    r["mm_r"] = (tr - test_tr0_) * der.mm_per_tick_r;
    r["yaw_deg"] = rad2deg(wrap(heading.psi - test_psi0_));
    event(e);
    test_side_ = 0;
    state = "IDLE";
  }
  hal.setMotor('l', c.motor_invert_l ? -duty_l : duty_l, b);
  hal.setMotor('r', c.motor_invert_r ? -duty_r : duty_r, b);
  // An toàn: duty lớn mà bánh quay NGƯỢC chiều lệnh → motor/encoder đấu ngược → PI phản hồi dương.
  const WheelController* wc[2] = {&drive.wl, &drive.wr};
  const float vm[2] = {drive.v_meas_l, drive.v_meas_r};
  for (int i = 0; i < 2; ++i) {
    const bool bad = std::fabs(wc[i]->duty) > 0.6f && wc[i]->vref * vm[i] < 0 && std::fabs(vm[i]) > 60.f;
    dir_t_[i] = bad ? dir_t_[i] + dt : 0.f;
    if (dir_t_[i] > 0.3f) {
      dir_t_[i] = 0;
      raiseFault(i == 0 ? "WHEEL_DIR_L" : "WHEEL_DIR_R");
    }
  }
}

void App::onRange(float l_mm, int l_st, float r_mm, int r_st) {
  raw_l = l_mm; raw_ls = l_st; raw_r = r_mm; raw_rs = r_st;
  const Config& c = params.active();
  meas = rangep.process(l_mm, l_st, r_mm, r_st, c, leader_rear_ >= 0 ? leader_rear_ : c.body_axle_to_rear_mm);
  meas.t_ms = hal.nowMs();
  has_meas = true;
  if (prim_ && state == "RUNNING" && !paused) prim_->onRange(*this, meas);
}

void App::onParams_(uint32_t groups) {
  if (groups & (G_RANGE | G_BODY)) {
    rangep.reset();
    if (kf.active && has_meas && meas.valid) kf.reset(meas.rho, kf.vL, params.active().range_sigma_mm);
  }
  if ((groups & G_MOTOR) && params.active().motor_pwm_hz != pwm_hz_) {
    pwm_hz_ = params.active().motor_pwm_hz;
    hal.setPwmFreq(pwm_hz_);
  }
  // PI bánh/hướng/follow: tích phân lưu theo đơn vị đầu ra → tự bumpless. Odometry giữ pose.
}

bool App::ledOn(uint32_t now) const {
  if (now < ident_until_) return (now / 120) % 2;
  if (state == "FAULT" || state == "ESTOP") return true;
  if (state == "CALIB") return (now / 60) % 2;
  if (state == "RUNNING") return (now / 250) % 2;
  if (!imu_ok_) return (now / 500) % 4 == 0;
  return (now / 1000) % 10 == 0;  // nháy thưa: sẵn sàng
}

// ================================================================ events
void App::event(JsonDocument& msg) {
  msg["eseq"] = ++eseq_;
  msg["ts"] = hal.nowMs();
  std::string s;
  serializeJson(msg, s);
  outbox_.push_back({eseq_, s});
  while (outbox_.size() > 64) outbox_.pop_front();
}

void App::raiseFault(const char* code) {
  if (state == "FAULT" || state == "ESTOP") return;
  if (prim_) {
    JsonDocument e;
    e["t"] = "evt"; e["e"] = "prim_done"; e["id"] = prim_->id; e["prim"] = prim_->name;
    e["ok"] = false; e["err"] = code; e["result"].to<JsonObject>();
    event(e);
    prim_.reset();
  }
  fault = code;
  state = "FAULT";
  JsonDocument e;
  e["t"] = "evt"; e["e"] = "fault"; e["code"] = code;
  event(e);
}

void App::finishPrim_() {
  std::unique_ptr<Prim> p = std::move(prim_);
  if (!std::strcmp(p->name, "FOLLOW")) kf.active = false;
  if (state == "RUNNING") state = p->from_idle ? "IDLE" : "ARMED";
  JsonDocument e;
  e["t"] = "evt"; e["e"] = "prim_done"; e["id"] = p->id; e["prim"] = p->name; e["ok"] = p->ok;
  if (p->err.empty()) e["err"] = nullptr; else e["err"] = p->err;
  if (p->result.isNull()) e["result"].to<JsonObject>();
  else e["result"] = p->result;
  event(e);
}

// ================================================================ IMU calib
bool App::startCalib_() {
  if (state != "IDLE" && state != "ARMED" && state != "FAULT" && state != "CALIB") return false;
  if (!imu_cal_) calib_prev_state_ = state;
  imu_cal_.reset(new ImuCalibrator(params.active()));
  state = "CALIB";
  return true;
}

void App::finishCalib_(const CalibResult& r) {
  imu_cal_.reset();
  if (state == "CALIB") state = calib_prev_state_;
  JsonDocument e;
  e["t"] = "evt"; e["e"] = "calib_done"; e["ok"] = r.ok;
  if (r.err) e["err"] = r.err; else e["err"] = nullptr;
  JsonObject res = e["result"].to<JsonObject>();
  res["restarts"] = r.restarts;
  if (r.ok) {
    heading.bias = r.bias;
    heading.anchorBias();
    imu_ok_ = true;
    res["bias_dps"] = rad2deg(r.bias);
    res["std_dps"] = rad2deg(r.stdev);
    res["spread_dps"] = rad2deg(r.spread);
    res["n"] = r.n;
    imu_info_.clear();
    imu_info_.set(res);
    imu_info_["t_ms"] = hal.nowMs();
  } else {
    imu_ok_ = false;
  }
  event(e);
}

// ================================================================ telemetry
void App::telemetry(JsonObject t) {
  const Config& c = params.active();
  const uint32_t now = hal.nowMs();
  t["t"] = "tel";
  t["id"] = id;
  t["t_ms"] = now;
  t["state"] = (paused && runState_()) ? "PAUSED" : state.c_str();
  if (role.empty()) t["role"] = nullptr; else t["role"] = role;
  t["cfg_rev"] = params.rev;
  if (fault.empty()) t["fault"] = nullptr; else t["fault"] = fault;
  JsonObject est = t["est"].to<JsonObject>();
  est["x"] = odo.x; est["y"] = odo.y; est["yaw"] = heading.psi; est["s"] = odo.s;
  est["v"] = drive.vMeas(); est["w"] = heading.w;
  JsonObject wh = t["wheels"].to<JsonObject>();
  JsonObject wl = wh["l"].to<JsonObject>();
  wl["v_cmd"] = drive.wl.vref; wl["v"] = drive.v_meas_l; wl["duty"] = drive.wl.duty; wl["adapt"] = drive.wl.adapt;
  JsonObject wr = wh["r"].to<JsonObject>();
  wr["v_cmd"] = drive.wr.vref; wr["v"] = drive.v_meas_r; wr["duty"] = drive.wr.duty; wr["adapt"] = drive.wr.adapt;
  JsonObject tof = t["tof"].to<JsonObject>();
  tof["l"]["mm"] = raw_l; tof["l"]["st"] = raw_ls;
  tof["r"]["mm"] = raw_r; tof["r"]["st"] = raw_rs;
  JsonObject gap = t["gap"].to<JsonObject>();
  if (kf.active) { gap["est"] = kf.g; gap["vL"] = kf.vL; } else { gap["est"] = nullptr; gap["vL"] = nullptr; }
  if (has_meas && meas.valid) gap["meas"] = meas.rho; else gap["meas"] = nullptr;
  const float gt = followTarget(prim_.get());
  if (gt >= 0) gap["target"] = gt; else gap["target"] = nullptr;
  if (has_meas && meas.has_dpsi) t["dpsi"] = meas.dpsi; else t["dpsi"] = nullptr;
  if (prim_) { t["prim"]["id"] = prim_->id; t["prim"]["name"] = prim_->name; } else t["prim"] = nullptr;
  JsonArray fl = t["flags"].to<JsonArray>();
  if (params.flag_default) fl.add("CFG_DEFAULT");
  if (params.flag_partial) fl.add("CFG_PARTIAL");
  if (params.dirty()) fl.add("CFG_DIRTY");
  if (bat_low_) fl.add("BAT_LOW");
  if (now < ident_until_) fl.add("IDENT");
  if (!imu_ok_) fl.add("IMU_UNCAL");
  if (imu_cal_) fl.add("IMU_CALIB");
  if (drive.sat) fl.add("SAT");
  if (std::fabs(heading.w_enc - heading.w_gyro) > 0.35f) fl.add("SLIP");
  const uint32_t hw = hal.hwFault();
  if (hw & HW_IMU) fl.add("HW_IMU");
  if (hw & HW_TOF_L) fl.add("HW_TOF_L");
  if (hw & HW_TOF_R) fl.add("HW_TOF_R");
  t["vbat"] = vbat_;
  t["gyro_bias"] = heading.bias;
  t["enc"]["l"] = last_tl_;  // tổng tick (đã áp enc.invert) — dùng cho hiệu chuẩn encoder quay tay
  t["enc"]["r"] = last_tr_;
  JsonObject imu = t["imu"].to<JsonObject>();
  imu["ok"] = imu_ok_;
  imu["bias_dps"] = rad2deg(heading.bias);
  if (imu_cal_) imu["calib"] = imu_cal_->progress(); else imu["calib"] = nullptr;
  imu["std_dps"] = imu_info_["std_dps"];
  imu["zupt"] = heading.zupt_active;
  imu["temp_c"] = heading.temp;
  imu["k_dps_c"] = rad2deg(heading.k(c));
  imu["k_learned"] = heading.has_k;
  if (imu_info_["t_ms"].is<uint32_t>()) imu["age_s"] = (now - imu_info_["t_ms"].as<uint32_t>()) / 1000.f;
  JsonObject g = t["geom"].to<JsonObject>();
  g["length"] = c.body_length_mm; g["width"] = c.body_width_mm; g["front"] = c.body_axle_to_front_mm;
  g["rear"] = c.body_axle_to_rear_mm; g["tof_x"] = c.range_x_mm; g["tof_b"] = c.range_baseline_mm;
  g["track"] = c.wheel_track_mm; g["wheel_d"] = c.wheel_diameter_mm;
}

// ================================================================ commands
void App::startPrim_(JsonObjectConst cmd, JsonObject out) {
  const char* pname = cmd["prim"] | "";
  std::unique_ptr<Prim> p = makePrim(pname);
  if (!p) { out["ok"] = false; out["err"] = "unknown_prim"; return; }
  const bool from_idle = state == "IDLE";
  if (from_idle) {  // hiệu chuẩn / kiểm tra: chạy từ IDLE, xong quay về IDLE
    if (!p->standalone) { out["ok"] = false; out["err"] = "state:IDLE"; return; }
    if (p->needs_imu && !imu_ok_) { out["ok"] = false; out["err"] = "imu_uncal"; return; }
  } else if (!runState_()) {
    out["ok"] = false; out["err"] = "state:" + state; return;
  }
  if (prim_) {
    JsonDocument e;
    e["t"] = "evt"; e["e"] = "prim_done"; e["id"] = prim_->id; e["prim"] = prim_->name;
    e["ok"] = false; e["err"] = "preempted"; e["result"].to<JsonObject>();
    event(e);
  }
  p->id = cmd["id"] | 0;
  p->from_idle = from_idle;
  for (JsonPairConst kv : cmd) {
    const char* k = kv.key().c_str();
    if (std::strcmp(k, "op") && std::strcmp(k, "id") && std::strcmp(k, "prim") && std::strcmp(k, "seq") && std::strcmp(k, "t"))
      p->args[k] = kv.value();
  }
  prim_ = std::move(p);
  state = "RUNNING";
  prim_->start(*this);
  out["ok"] = true;
  if (prim_->done) finishPrim_();
}

void App::handleCmd(JsonObjectConst cmd, JsonObject out) {
  const char* op = cmd["op"] | "";
  const uint32_t now = hal.nowMs();
  out["ok"] = true;
  if (!std::strcmp(op, "estop")) {
    prim_.reset(); imu_cal_.reset(); paused = false; test_side_ = 0; state = "ESTOP";
  } else if (!std::strcmp(op, "reset")) {
    if (state == "ESTOP" || state == "FAULT") { state = "IDLE"; fault.clear(); paused = false; prim_.reset(); }
  } else if (state == "ESTOP") {
    out["ok"] = false; out["err"] = "estop";
  } else if (!std::strcmp(op, "arm")) {
    if (!has_path_) { out["ok"] = false; out["err"] = "no_path"; }
    else if (state == "FAULT") { out["ok"] = false; out["err"] = "fault:" + fault; }
    else if (params.flag_default) { out["ok"] = false; out["err"] = "cfg_default"; }
    else if (imu_cal_) { out["ok"] = false; out["err"] = "imu_calibrating"; }
    else if (params.active().imu_require_calib && !imu_ok_) { out["ok"] = false; out["err"] = "imu_uncal"; }
    else if (hal.hwFault() & HW_IMU) { out["ok"] = false; out["err"] = "hw:imu"; }
    else if (role == "follower" && (hal.hwFault() & (HW_TOF_L | HW_TOF_R))) { out["ok"] = false; out["err"] = "hw:tof"; }
    else { state = "ARMED"; paused = false; last_hb_ms_ = now; }
  } else if (!std::strcmp(op, "prim")) {
    startPrim_(cmd, out);
  } else if (!std::strcmp(op, "pause")) {
    paused = true;
  } else if (!std::strcmp(op, "resume")) {
    paused = false; last_hb_ms_ = now;
  } else if (!std::strcmp(op, "stop")) {
    prim_.reset(); imu_cal_.reset(); paused = false; test_side_ = 0;
    if (state == "TEST") state = "IDLE";
    if (state != "FAULT") state = "IDLE";
  } else if (!std::strcmp(op, "motor_test")) {
    const char* side = cmd["side"] | "";
    const float duty = clampf(cmd["duty"] | 0.4f, -0.8f, 0.8f);
    const uint32_t ms = std::min<uint32_t>(cmd["ms"] | 500u, 2000u);
    if (state != "IDLE") { out["ok"] = false; out["err"] = "state:" + state; }
    else if (std::strcmp(side, "l") && std::strcmp(side, "r")) { out["ok"] = false; out["err"] = "side phải là l|r"; }
    else {
      test_side_ = side[0];
      test_duty_ = duty;
      test_until_ = now + ms;
      test_tl0_ = last_tl_;
      test_tr0_ = last_tr_;
      test_psi0_ = heading.psi;
      state = "TEST";
    }
  } else if (!std::strcmp(op, "calib_imu")) {
    if (!startCalib_()) { out["ok"] = false; out["err"] = "state:" + state; }
  } else if (!std::strcmp(op, "set_s")) {
    odo.s = cmd["s"] | odo.s;
  } else if (!std::strcmp(op, "jog")) {
    if (state != "IDLE" && state != "JOG") { out["ok"] = false; out["err"] = "state:" + state; }
    else { state = "JOG"; jog_v_ = cmd["v"] | 0.f; jog_w_ = cmd["w"] | 0.f; jog_until_ = now + 300; }
  } else {
    out["ok"] = false; out["err"] = "unknown_op";
  }
  out["state"] = state;
}

// ================================================================ HTTP
void App::configView_(JsonObject o) {
  configToJson(params.view(), o["params"].to<JsonObject>());
  derivedToJson(params.viewDerived(), o["derived"].to<JsonObject>());
  o["cfg_rev"] = params.rev;
  o["crc"] = configCrc(params.view());
  if (params.saved_crc.empty()) o["saved_crc"] = nullptr; else o["saved_crc"] = params.saved_crc;
  o["dirty"] = params.dirty();
  JsonArray f = o["flags"].to<JsonArray>();
  if (params.flag_default) f.add("CFG_DEFAULT");
  if (params.flag_partial) f.add("CFG_PARTIAL");
}

int App::loadPath_(JsonObjectConst p, JsonDocument& out) {
  if (running_()) { out["ok"] = false; out["err"] = "đang chạy"; return 409; }
  if (role != "leader" && role != "follower") { out["ok"] = false; out["err"] = "chưa có vai trò"; return 400; }
  JsonObjectConst st = role == "leader" ? p["start"].as<JsonObjectConst>() : p["follower"]["start"].as<JsonObjectConst>();
  if (st.isNull()) { out["ok"] = false; out["err"] = "path thiếu start"; return 400; }
  has_path_ = true;
  path_id_ = p["path_id"] | 0;
  path_crc_ = p["crc32"] | "";
  leader_rear_ = p["formation"]["leader_rear_mm"] | -1.f;
  const float s0 = role == "leader" ? 0.f : (p["follower"]["start_s"] | 0.f);
  odo.reset(st["x"] | 0.f, st["y"] | 0.f, s0);
  heading.psi = deg2rad(st["yaw_deg"] | 0.f);
  kf.active = false;
  rangep.reset();
  has_last_ticks_ = false;
  if (state == "FAULT") { state = "IDLE"; fault.clear(); }
  out["ok"] = true;
  out["path_id"] = path_id_;
  out["crc"] = path_crc_;
  return 200;
}

int App::handleRequest(const char* method, const std::string& path, JsonObjectConst body, JsonDocument& out) {
  last_hb_ms_ = hal.nowMs();  // mọi request từ PC là heartbeat
  link_watch_ = true;
  const bool get = !std::strcmp(method, "GET"), put = !std::strcmp(method, "PUT"), post = !std::strcmp(method, "POST");
  const bool running = running_();

  if (get && path == "/api/info") {
    out["id"] = id; out["name"] = name; out["fw"] = FW_VERSION;
    if (role.empty()) out["role"] = nullptr; else out["role"] = role;
    out["state"] = state; out["cfg_rev"] = params.rev; out["schema_ver"] = SCHEMA_VER;
    return 200;
  }
  if (get && path == "/api/state") {
    const uint32_t since = body["since"] | 0u;
    telemetry(out["tel"].to<JsonObject>());
    JsonArray ev = out["events"].to<JsonArray>();
    for (auto& e : outbox_)
      if (e.eseq > since) ev.add(serialized(e.json));
    out["eseq"] = eseq_;
    out["events_lost"] = !outbox_.empty() && since + 1 < outbox_.front().eseq;
    return 200;
  }
  if (post && path == "/api/cmd") {
    handleCmd(body, out.to<JsonObject>());
    return out["ok"] ? 200 : 409;
  }
  if (post && path == "/api/relay") {
    relay_valid = true;
    relay_t = hal.nowMs();
    relay_v = body["v"] | 0.f;
    relay_s = body["s"] | 0.f;
    out["ok"] = true;
    return 200;
  }
  if (path == "/api/config") {
    if (get) { configView_(out.to<JsonObject>()); return 200; }
    if (put) {
      JsonObject o = out.to<JsonObject>();
      params.submit(body["patch"].as<JsonObjectConst>(), running, o);
      if (o["ok"] && (body["persist"] | false)) params.save(o["saved"].to<JsonObject>());
      return o["ok"] ? 200 : (o["busy"] | false) ? 409 : 400;
    }
  }
  if (post && path == "/api/config/save") { params.save(out.to<JsonObject>()); return 200; }
  if (post && path == "/api/config/reload") {
    params.reload(running, out.to<JsonObject>());
    return out["ok"] ? 200 : 400;
  }
  if (post && path == "/api/config/factory_reset") {
    params.factoryReset(running, out.to<JsonObject>());
    return out["ok"] ? 200 : 400;
  }
  if (put && path == "/api/role") {
    JsonVariantConst r = body["role"];
    std::string nr = r.isNull() ? "" : std::string(r.as<const char*>() ? r.as<const char*>() : "");
    if (!nr.empty() && nr != "leader" && nr != "follower") { out["ok"] = false; out["err"] = "role không hợp lệ"; return 400; }
    if (running) { out["ok"] = false; out["err"] = "đang chạy"; return 409; }
    role = nr;
    hal.nvsSetRole(role);
    out["ok"] = true;
    if (role.empty()) out["role"] = nullptr; else out["role"] = role;
    return 200;
  }
  if (put && path == "/api/path") return loadPath_(body, out);
  if (post && path == "/api/identify") { ident_until_ = hal.nowMs() + 2500; out["ok"] = true; return 200; }
  out["ok"] = false;
  out["err"] = std::string("không có route ") + method + " " + path;
  return 404;
}

}  // namespace tr

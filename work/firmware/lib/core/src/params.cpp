#include "params.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "mathutil.h"

namespace tr {

static inline uint8_t* fieldPtr(Config& c, const ParamMeta& m) { return reinterpret_cast<uint8_t*>(&c) + m.offset; }
static inline const uint8_t* fieldPtr(const Config& c, const ParamMeta& m) {
  return reinterpret_cast<const uint8_t*>(&c) + m.offset;
}

const ParamMeta* findParam(const char* key) {
  for (size_t i = 0; i < PARAM_COUNT; ++i)
    if (std::strcmp(PARAMS[i].key, key) == 0) return &PARAMS[i];
  return nullptr;
}

float getNum(const Config& c, const ParamMeta& m) {
  const uint8_t* p = fieldPtr(c, m);
  switch (m.type) {
    case PType::Float: return *reinterpret_cast<const float*>(p);
    case PType::Int: return static_cast<float>(*reinterpret_cast<const int32_t*>(p));
    case PType::Bool: return *reinterpret_cast<const bool*>(p) ? 1.f : 0.f;
  }
  return 0;
}

bool getBool(const Config& c, const ParamMeta& m) { return getNum(c, m) != 0.f; }

static bool sameValue(const Config& a, const Config& b, const ParamMeta& m) {
  size_t n = m.type == PType::Bool ? sizeof(bool) : 4;
  return std::memcmp(fieldPtr(a, m), fieldPtr(b, m), n) == 0;
}

static std::string fmtNum(float v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
  return buf;
}

// Ép kiểu + kiểm tra min/max (thông báo tiếng Việt giống params.py)
static bool coerceSet(Config& c, const ParamMeta& m, JsonVariantConst v, std::string& err) {
  uint8_t* p = fieldPtr(c, m);
  if (m.type == PType::Bool) {
    if (v.is<bool>()) { *reinterpret_cast<bool*>(p) = v.as<bool>(); return true; }
    if (v.is<int>() && (v.as<int>() == 0 || v.as<int>() == 1)) { *reinterpret_cast<bool*>(p) = v.as<int>() == 1; return true; }
    if (v.is<const char*>()) {
      const char* s = v.as<const char*>();
      if (!std::strcmp(s, "true") || !std::strcmp(s, "1")) { *reinterpret_cast<bool*>(p) = true; return true; }
      if (!std::strcmp(s, "false") || !std::strcmp(s, "0")) { *reinterpret_cast<bool*>(p) = false; return true; }
    }
    err = "phải là true/false";
    return false;
  }
  if (v.is<bool>() || !(v.is<float>() || v.is<const char*>())) { err = "phải là số"; return false; }
  float x;
  if (v.is<const char*>()) {
    char* end = nullptr;
    x = std::strtof(v.as<const char*>(), &end);
    if (end == v.as<const char*>()) { err = "phải là số"; return false; }
  } else {
    x = v.as<float>();
  }
  if (!std::isfinite(x)) { err = "phải là số hữu hạn"; return false; }
  if (m.type == PType::Int) x = std::round(x);
  if (x < m.min) { err = "phải ≥ " + fmtNum(m.min); return false; }
  if (x > m.max) { err = "phải ≤ " + fmtNum(m.max); return false; }
  if (m.type == PType::Int) *reinterpret_cast<int32_t*>(p) = static_cast<int32_t>(x);
  else *reinterpret_cast<float*>(p) = x;
  return true;
}

void crossCheck(const Config& c, Errors& e) {
  if (c.range_min_valid_mm >= c.range_max_valid_mm) e["range.min_valid_mm"] = "phải nhỏ hơn range.max_valid_mm";
  if (c.body_axle_to_front_mm + c.body_axle_to_rear_mm > c.body_length_mm + 1.f)
    e["body.length_mm"] = "phải ≥ (tâm trục→trước) + (tâm trục→sau)";
  if (c.range_x_mm > c.body_axle_to_front_mm + 1.f) e["range.x_mm"] = "mắt cảm biến nằm ngoài mặt trước thân xe";
  if (c.range_baseline_mm > c.body_width_mm) e["range.baseline_mm"] = "khoảng cách 2 mắt lớn hơn bề rộng thân";
  if (c.wheel_track_mm > c.body_width_mm + 60.f) e["wheel.track_mm"] = "track lớn bất thường so với bề rộng thân";
}

bool validatePatch(const Config& base, JsonObjectConst patch, Config& out, Errors& errs) {
  out = base;
  for (JsonPairConst kv : patch) {
    const ParamMeta* m = findParam(kv.key().c_str());
    if (!m) { errs[kv.key().c_str()] = "khoá không tồn tại"; continue; }
    std::string err;
    if (!coerceSet(out, *m, kv.value(), err)) errs[m->key] = err;
  }
  if (errs.empty()) crossCheck(out, errs);
  return errs.empty();
}

uint32_t changedGroups(const Config& a, const Config& b, std::vector<std::string>* idleKeys) {
  uint32_t mask = 0;
  for (size_t i = 0; i < PARAM_COUNT; ++i) {
    const ParamMeta& m = PARAMS[i];
    if (sameValue(a, b, m)) continue;
    mask |= m.group;
    if (idleKeys && m.idle) idleKeys->push_back(m.key);
  }
  return mask;
}

float sweepRadius(const Config& c) {
  return std::hypot(std::max(c.body_axle_to_front_mm, c.body_axle_to_rear_mm), c.body_width_mm / 2.f);
}

float wheelVmax(const Config& c, float vbat) {
  float vavail = vbat - c.power_l298n_vdrop_v;
  float vl = (vavail - c.motor_ks_l_v) / c.motor_kv_l;
  float vr = (vavail - c.motor_ks_r_v) / c.motor_kv_r;
  return std::max(0.f, std::min(vl, vr));
}

Derived derive(const Config& c) {
  Derived d;
  d.ticks_per_rev = static_cast<float>(c.enc_ppr_motor) * 4.f * c.enc_gear_ratio;
  d.mm_per_tick_l = PI_F * c.wheel_diameter_mm * (1.f + c.wheel_diam_eps) / d.ticks_per_rev;
  d.mm_per_tick_r = PI_F * c.wheel_diameter_mm * (1.f - c.wheel_diam_eps) / d.ticks_per_rev;
  d.r_sweep_mm = sweepRadius(c);
  d.v_wheel_max_mm_s = wheelVmax(c);
  return d;
}

void configToJson(const Config& c, JsonObject out) {
  for (size_t i = 0; i < PARAM_COUNT; ++i) {
    const ParamMeta& m = PARAMS[i];
    switch (m.type) {
      case PType::Float: out[m.key] = getNum(c, m); break;
      case PType::Int: out[m.key] = static_cast<int32_t>(getNum(c, m)); break;
      case PType::Bool: out[m.key] = getBool(c, m); break;
    }
  }
}

void derivedToJson(const Derived& d, JsonObject out) {
  out["ticks_per_rev"] = d.ticks_per_rev;
  out["mm_per_tick_l"] = d.mm_per_tick_l;
  out["mm_per_tick_r"] = d.mm_per_tick_r;
  out["r_sweep_mm"] = d.r_sweep_mm;
  out["v_wheel_max_mm_s"] = d.v_wheel_max_mm_s;
}

void groupsToJson(uint32_t mask, JsonArray out) {
  for (size_t i = 0; i < GROUP_COUNT; ++i)
    if (mask & (1u << i)) out.add(GROUP_NAMES[i]);
}

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

std::string configCrc(const Config& c) {
  JsonDocument doc;
  configToJson(c, doc.to<JsonObject>());
  std::string s;
  serializeJson(doc, s);
  char buf[9];
  std::snprintf(buf, sizeof(buf), "%08x", static_cast<unsigned>(crc32(reinterpret_cast<const uint8_t*>(s.data()), s.size())));
  return buf;
}

}  // namespace tr

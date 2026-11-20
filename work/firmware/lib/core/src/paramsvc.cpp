#include "paramsvc.h"

namespace tr {

void ParamService::load() {
  Config base;
  std::string json;
  JsonDocument doc;
  if (hal_.storageLoad(json) && !deserializeJson(doc, json) && doc["params"].is<JsonObjectConst>()) {
    JsonObjectConst stored = doc["params"].as<JsonObjectConst>();
    // Bỏ khoá lạ, giữ phần hợp lệ (schema có thể đã đổi phiên bản)
    JsonDocument known;
    JsonObject k = known.to<JsonObject>();
    for (JsonPairConst kv : stored)
      if (findParam(kv.key().c_str())) k[kv.key()] = kv.value();
    Config out;
    Errors errs;
    if (validatePatch(base, k, out, errs)) {
      base = out;
    } else {
      JsonDocument good;
      JsonObject g = good.to<JsonObject>();
      for (JsonPair kv : k)
        if (!errs.count(kv.key().c_str())) g[kv.key()] = kv.value();
      Errors errs2;
      if (validatePatch(base, g, out, errs2)) { base = out; flag_partial = true; }
      else flag_default = true;
    }
    rev = doc["rev"] | 1u;
    saved_crc = configCrc(base);
  } else {
    flag_default = true;
  }
  active_ = base;
  derived_ = derive(base);
  view_crc_ = configCrc(base);
}

void ParamService::submit(JsonObjectConst patch, bool running, JsonObject out) {
  const Config& base = view();
  Config nw;
  Errors errs;
  if (!validatePatch(base, patch, nw, errs)) {
    out["ok"] = false;
    JsonObject e = out["errors"].to<JsonObject>();
    for (auto& kv : errs) e[kv.first] = kv.second;
    return;
  }
  std::vector<std::string> idle;
  const uint32_t groups = changedGroups(base, nw, &idle);
  if (running) {
    if (!idle.empty()) {
      out["ok"] = false;
      out["busy"] = true;
      JsonObject e = out["errors"].to<JsonObject>();
      for (auto& k : idle) e[k] = "chỉ đổi được khi xe đứng yên";
      return;
    }
    if (!nw.safety_allow_tune_while_running || !base.safety_allow_tune_while_running) {
      out["ok"] = false;
      out["busy"] = true;
      out["errors"]["*"] = "đang chạy, không cho chỉnh live";
      return;
    }
  }
  out["ok"] = true;
  if (!groups) {
    out["cfg_rev"] = rev + (has_pending_ ? 1 : 0);
    out["changed"].to<JsonArray>();
    return;
  }
  pending_ = nw;
  pending_der_ = derive(nw);
  view_crc_ = configCrc(nw);
  pending_groups_ |= groups;
  has_pending_ = true;
  out["cfg_rev"] = rev + 1;
  groupsToJson(groups, out["changed"].to<JsonArray>());
}

uint32_t ParamService::swapIfPending() {
  if (!has_pending_) return 0;
  active_ = pending_;
  derived_ = pending_der_;
  has_pending_ = false;
  const uint32_t g = pending_groups_;
  pending_groups_ = 0;
  ++rev;
  flag_default = flag_partial = false;
  return g;
}

void ParamService::save(JsonObject out) {
  const Config& c = view();
  const uint32_t r = rev + (has_pending_ ? 1 : 0);
  JsonDocument doc;
  doc["schema_ver"] = SCHEMA_VER;
  doc["rev"] = r;
  configToJson(c, doc["params"].to<JsonObject>());
  std::string s;
  serializeJson(doc, s);
  const bool ok = hal_.storageSave(s);
  if (ok) {
    saved_crc = view_crc_;
    flag_default = flag_partial = false;  // người dùng đã xác nhận cấu hình
  }
  out["ok"] = ok;
  out["cfg_rev"] = r;
  out["crc"] = saved_crc;
}

void ParamService::reload(bool running, JsonObject out) {
  std::string json;
  JsonDocument doc;
  if (!hal_.storageLoad(json) || deserializeJson(doc, json)) {
    out["ok"] = false;
    out["errors"]["*"] = "flash chưa có cấu hình";
    return;
  }
  submit(doc["params"].as<JsonObjectConst>(), running, out);
}

void ParamService::factoryReset(bool running, JsonObject out) {
  JsonDocument doc;
  configToJson(Config(), doc.to<JsonObject>());
  submit(doc.as<JsonObjectConst>(), running, out);
}

}  // namespace tr

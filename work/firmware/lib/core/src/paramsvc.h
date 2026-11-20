#pragma once
// ParamService: tham số động lúc chạy (wiki/config-management.md §3). Tương ứng fw/paramsvc.py.
// patch → validate → derive → slot pending → control task swap ở đầu chu kỳ.
#include <ArduinoJson.h>

#include <string>

#include "hal.h"
#include "params.h"

namespace tr {

class ParamService {
 public:
  explicit ParamService(Hal& hal) : hal_(hal) {}
  void load();  // gọi 1 lần lúc khởi động

  const Config& active() const { return active_; }
  const Derived& derived() const { return derived_; }
  const Config& view() const { return has_pending_ ? pending_ : active_; }
  const Derived& viewDerived() const { return has_pending_ ? pending_der_ : derived_; }

  uint32_t rev = 1;
  std::string saved_crc;
  bool flag_default = false, flag_partial = false;

  bool dirty() const { return configCrc(view()) != saved_crc; }
  void submit(JsonObjectConst patch, bool running, JsonObject out);
  uint32_t swapIfPending();  // trả bitmask nhóm vừa đổi (0 = không đổi)
  void save(JsonObject out);
  void reload(bool running, JsonObject out);
  void factoryReset(bool running, JsonObject out);

 private:
  Hal& hal_;
  Config active_, pending_;
  Derived derived_, pending_der_;
  bool has_pending_ = false;
  uint32_t pending_groups_ = 0;
};

}  // namespace tr

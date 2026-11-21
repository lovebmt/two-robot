#pragma once
// Primitive chuyển động — PC chỉ ra lệnh ở mức này. Tương ứng fw/primitives.py.
// start(app) → update(app, dt) trả (v, ω, brake) mỗi chu kỳ 100 Hz; onRange khi có mẫu ToF.
#include <ArduinoJson.h>

#include <memory>
#include <string>
#include <vector>

#include "rangeproc.h"

namespace tr {

class App;

struct Cmd {
  float v = 0, w = 0;
  bool brake = true;
  bool raw = false;              // raw: điện áp thẳng xuống 2 motor (hiệu chuẩn CAL_KS), bỏ qua PI bánh
  float volt_l = 0, volt_r = 0;  // V, dấu = chiều tiến
};
constexpr Cmd BRAKE{0, 0, true};

class Prim {
 public:
  int id = 0;
  const char* name = "?";
  bool standalone = false;  // chạy được cả khi IDLE (hiệu chuẩn/kiểm tra)
  bool needs_imu = true;    // khi chạy từ IDLE: cần gyro đã calib
  bool from_idle = false;
  bool done = false, ok = true;
  std::string err;
  JsonDocument args;
  JsonDocument result;

  virtual ~Prim() {}
  virtual void start(App&) {}
  virtual Cmd update(App&, float) { return BRAKE; }
  virtual void onRange(App&, const RangeMeas&) {}

  void finish(bool ok_ = true, const char* err_ = nullptr) {
    done = true;
    ok = ok_;
    if (err_) err = err_;
  }
  float argf(const char* k, float def) const {
    JsonVariantConst v = args[k];
    return v.is<float>() ? v.as<float>() : def;
  }
  bool has(const char* k) const { return args[k].is<float>(); }
};

std::unique_ptr<Prim> makePrim(const char* name);
float followTarget(const Prim* p);  // g_target nếu p là FOLLOW, ngược lại -1

// Fit hiệu chuẩn 2 mắt ToF (xem TofCal). Mẫu: (θ, d_l thô, d_r thô).
struct TofSample {
  float th, dl, dr;
};
bool tofFit(const std::vector<TofSample>& s, JsonObject out);

}  // namespace tr

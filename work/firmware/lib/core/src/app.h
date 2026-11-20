#pragma once
// App: ứng dụng firmware chạy trên HAL bất kỳ. Tương ứng fw/robot.py (VirtualFirmware).
//  - tick(dt) 100 Hz (task control): swap tham số, đo, ước lượng, primitive, PWM, an toàn
//  - onRange(...) ~30 Hz khi có mẫu ToF
//  - handleRequest(...) HTTP API (wiki/protocol.md §2) — gọi trong mutex cùng tick()
#include <ArduinoJson.h>

#include <deque>
#include <memory>
#include <string>

#include "control.h"
#include "gapkf.h"
#include "hal.h"
#include "paramsvc.h"
#include "primitives.h"
#include "rangeproc.h"

namespace tr {

extern const char* FW_VERSION;

class App {
 public:
  App(Hal& hal, const std::string& id, const std::string& name);
  void begin();  // nạp config + vai trò từ flash/NVS

  void tick(float dt);
  void onRange(float l_mm, int l_st, float r_mm, int r_st);

  // HTTP: method "GET"/"PUT"/"POST"; body là JSON đã parse (GET: query, vd {"since": N}).
  int handleRequest(const char* method, const std::string& path, JsonObjectConst body, JsonDocument& out);
  void handleCmd(JsonObjectConst cmd, JsonObject out);
  void telemetry(JsonObject out);

  // ---- dùng bởi primitive ----
  Hal& hal;
  ParamService params;
  DriveBase drive;
  HeadingEstimator heading;
  Odometry odo;
  GapKF kf;
  RangeProcessor rangep;
  bool has_meas = false;
  RangeMeas meas;
  float raw_l = 0, raw_r = 0;
  int raw_ls = 4, raw_rs = 4;
  bool relay_valid = false;
  uint32_t relay_t = 0;
  float relay_v = 0, relay_s = 0;
  void raiseFault(const char* code);
  void event(JsonDocument& msg);  // thêm eseq, đưa vào hộp thư

  // ---- trạng thái ----
  std::string id, name, role, state = "IDLE", fault;
  bool paused = false;
  bool ledOn(uint32_t now) const;

 private:
  std::unique_ptr<Prim> prim_;
  std::unique_ptr<ImuCalibrator> imu_cal_;
  bool imu_ok_ = false;
  JsonDocument imu_info_;
  std::string calib_prev_state_ = "IDLE";
  float enc_still_t_ = 0;
  float dir_t_[2] = {0, 0};  // thời gian bánh quay ngược chiều lệnh khi duty lớn
  bool has_last_ticks_ = false;
  int32_t last_tl_ = 0, last_tr_ = 0;
  uint32_t last_hb_ms_ = 0;
  bool link_watch_ = false;
  float jog_v_ = 0, jog_w_ = 0;
  uint32_t jog_until_ = 0, ident_until_ = 0;
  float v_cmd_ = 0, w_cmd_ = 0, vbat_ = 12.f;
  bool bat_low_ = false;
  int pwm_hz_ = 0;
  // đường đi
  bool has_path_ = false;
  int path_id_ = 0;
  std::string path_crc_;
  float leader_rear_ = -1;
  // hộp thư sự kiện cho GET /api/state?since=
  struct Ev { uint32_t eseq; std::string json; };
  std::deque<Ev> outbox_;
  uint32_t eseq_ = 0;

  bool running_() const { return state == "RUNNING" || state == "PAUSED" || state == "JOG" || paused; }
  bool runState_() const { return state == "ARMED" || state == "RUNNING" || state == "PAUSED"; }
  void startPrim_(JsonObjectConst cmd, JsonObject out);
  void finishPrim_();
  bool startCalib_();
  void finishCalib_(const CalibResult& r);
  void onParams_(uint32_t groups);
  int loadPath_(JsonObjectConst p, JsonDocument& out);
  void configView_(JsonObject out);
};

}  // namespace tr

// two-robot firmware ESP32 — xem work/firmware/README.md
//
// Task (FreeRTOS):
//   control  core 1, 100 Hz, ưu tiên 5: encoder, gyro, ước lượng, primitive, PWM, an toàn
//   range    core 1, ~30 Hz, ưu tiên 3: đọc 2 ToF (chặn ~33 ms) → hộp thư cho control
//   http     core 0, ưu tiên 2:        WebServer (route /api/* → App, khoá chung mutex)
//   loop     core 1, ưu tiên 1:        console USB + telnet :23, OTA :3232
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "app.h"
#include "console.h"
#include "board/pins.h"
#include "hal_esp32.h"
#include "net.h"

static Esp32Hal hal;
static tr::App* app = nullptr;
static SemaphoreHandle_t appMutex = nullptr;

// Hộp thư ToF: range task ghi, control task đọc (spinlock vì 2 task cùng core nhưng khác ưu tiên)
struct TofSample { float l, r; int ls, rs; uint32_t seq; };
static TofSample tofBox = {8190, 8190, 4, 4, 0};
static portMUX_TYPE tofMux = portMUX_INITIALIZER_UNLOCKED;

static void rangeTask(void*) {
  uint32_t seq = 0;
  for (;;) {
    TofSample s;
    hal.tof.read(s.l, s.ls, s.r, s.rs);
    s.seq = ++seq;
    portENTER_CRITICAL(&tofMux);
    tofBox = s;
    portEXIT_CRITICAL(&tofMux);
  }
}

// Snapshot telemetry + sự kiện cho GET /api/state: task control dựng mỗi 40 ms, HTTP chỉ sao chép
// (khoá riêng giữ vài µs) → HTTP không bao giờ chặn vòng điều khiển 100 Hz.
static SemaphoreHandle_t snapMutex = nullptr;
static std::string snapTel;
static std::deque<tr::App::Ev> snapEv;
static uint32_t snapEseq = 0;

void stateSnapshot(std::string& tel, std::deque<tr::App::Ev>& ev, uint32_t& eseq) {
  xSemaphoreTake(snapMutex, portMAX_DELAY);
  tel = snapTel;
  ev = snapEv;
  eseq = snapEseq;
  xSemaphoreGive(snapMutex);
}

static void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  uint32_t lastSeq = 0, maxUs = 0, lastLog = 0, n = 0;
  uint32_t mxLock = 0, mxTick = 0, mxSnap = 0, mxSnapLock = 0, mxRange = 0;
  std::string tel;
  std::deque<tr::App::Ev> ev;
  uint32_t eseq = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    const uint32_t t0 = micros();
    TofSample s;
    portENTER_CRITICAL(&tofMux);
    s = tofBox;
    portEXIT_CRITICAL(&tofMux);
    xSemaphoreTake(appMutex, portMAX_DELAY);
    const uint32_t t1 = micros();
    if (s.seq != lastSeq) {
      lastSeq = s.seq;
      app->onRange(s.l, s.ls, s.r, s.rs);
    }
    const uint32_t t2 = micros();
    app->tick(0.01f);
    const uint32_t t3 = micros();
    const bool led = app->ledOn(millis());
    const bool snap = (++n % 4) == 0;  // 25 Hz
    if (snap) app->snapshot(tel, ev, eseq);
    const uint32_t t4 = micros();
    xSemaphoreGive(appMutex);
    mxLock = std::max(mxLock, t1 - t0);
    mxRange = std::max(mxRange, t2 - t1);
    mxTick = std::max(mxTick, t3 - t2);
    mxSnap = std::max(mxSnap, t4 - t3);
    if (snap) {
      const uint32_t t5 = micros();
      xSemaphoreTake(snapMutex, portMAX_DELAY);
      mxSnapLock = std::max(mxSnapLock, (uint32_t)(micros() - t5));
      std::swap(snapTel, tel);
      std::swap(snapEv, ev);
      snapEseq = eseq;
      xSemaphoreGive(snapMutex);
    }
    hal.setLed(led);
    const uint32_t us = micros() - t0;
    if (us > maxUs) maxUs = us;
    if (millis() - lastLog > 10000) {  // nhật ký nhẹ mỗi 10 s
      lastLog = millis();
      con.printf("[%lus] state=%s role=%s ip=%s tick_max=%luus (khoá %lu, tof %lu, tick %lu, snap %lu, khoá snap %lu) heap=%u hw=0x%x\n",
                 millis() / 1000, app->state.c_str(), app->role.c_str(), net::ipString().c_str(), (unsigned long)maxUs,
                 (unsigned long)mxLock, (unsigned long)mxRange, (unsigned long)mxTick, (unsigned long)mxSnap,
                 (unsigned long)mxSnapLock, ESP.getFreeHeap(), (unsigned)hal.hwFault());
      maxUs = mxLock = mxRange = mxTick = mxSnap = mxSnapLock = 0;
    }
  }
}

void setup() {
  // ĐẦU TIÊN: khoá motor (EN = 0, IN = 0) trước mọi việc khác.
  const uint8_t motorPins[] = {pins::MOTOR_L_EN, pins::MOTOR_R_EN, pins::MOTOR_L_IN1, pins::MOTOR_L_IN2,
                               pins::MOTOR_R_IN1, pins::MOTOR_R_IN2};
  for (uint8_t p : motorPins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== two-robot fw %s ===\n", tr::FW_VERSION);
  hal.begin();  // motor tắt trước tiên

  char id[7];
  const uint64_t mac = ESP.getEfuseMac();
  snprintf(id, sizeof(id), "%02x%02x%02x", (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
  String name = hal.prefs.isKey("name") ? hal.prefs.getString("name", "") : String("Robot-") + id;

  app = new tr::App(hal, id, name.c_str());
  app->begin();
  appMutex = xSemaphoreCreateMutex();
  snapMutex = xSemaphoreCreateMutex();
  Serial.printf("id=%s name=%s role=%s cfg_rev=%u%s\n", id, name.c_str(), app->role.c_str(), app->params.rev,
                app->params.flag_default ? " (CẤU HÌNH MẶC ĐỊNH — cần Save trên Web Control trước khi chạy)" : "");

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(rangeTask, "range", 4096, nullptr, 3, nullptr, 1);

  net::begin(hal.prefs, id);
  net::startHttp(app, appMutex, &hal.prefs);
  xTaskCreatePinnedToCore(net::httpTask, "http", 12288, nullptr, 2, nullptr, 0);
}

// ================================================================ Console (USB serial + telnet WiFi :23)
//   status            số đo ToF, gyro, nhiệt độ, encoder, trạng thái
//   watch             bật/tắt in status 2 lần/giây
//   wifi <ssid> <pw>  lưu WiFi vào NVS rồi khởi động lại (pw có thể bỏ trống)
//   reboot | help
static WiFiServer telnetServer(23);
static WiFiClient telnet;
static bool watchOn = false;

static void printStatus() {
  xSemaphoreTake(appMutex, portMAX_DELAY);
  const float rl = app->raw_l, rr = app->raw_r;
  const int sl = app->raw_ls, sr = app->raw_rs;
  const float psi = app->heading.psi * 57.29578f, bias = app->heading.bias * 57.29578f, temp = app->heading.temp;
  const float s = app->odo.s, vl = app->drive.v_meas_l, vr = app->drive.v_meas_r;
  const std::string st = app->state, fault = app->fault;
  xSemaphoreGive(appMutex);
  const int32_t el = hal.readEncoderRaw('l'), er = hal.readEncoderRaw('r');
  con.printf("ToF trái %s%.0f mm · phải %s%.0f mm | gyro hướng %.1f° bias %.2f°/s · %.1f°C | "
             "enc tick L %ld R %ld · v L %.0f R %.0f mm/s · s %.0f mm | %s%s%s\n",
             sl ? "×" : "", rl, sr ? "×" : "", rr, psi, bias, temp, (long)el, (long)er, vl, vr, s, st.c_str(),
             fault.empty() ? "" : " lỗi ", fault.c_str());
}

static void handleLine(String line) {
  line.trim();
  if (line == "status") {
    printStatus();
  } else if (line == "watch") {
    watchOn = !watchOn;
    con.printf("watch %s\n", watchOn ? "BẬT (gõ watch lần nữa để tắt)" : "TẮT");
  } else if (line.startsWith("wifi ")) {
    String rest = line.substring(5);
    rest.trim();
    const int sp = rest.indexOf(' ');
    const String ssid = sp < 0 ? rest : rest.substring(0, sp);
    const String pass = sp < 0 ? String() : rest.substring(sp + 1);
    hal.prefs.putString("ssid", ssid);
    hal.prefs.putString("pass", pass);
    con.printf("Đã lưu WiFi \"%s\". Khởi động lại…\n", ssid.c_str());
    delay(300);
    ESP.restart();
  } else if (line == "reboot") {
    con.println("Khởi động lại…");
    delay(200);
    ESP.restart();
  } else if (line.length()) {
    con.printf("two-robot %s · id %s · %s\nLệnh: status | watch | wifi <ssid> <mật khẩu> | reboot\n", tr::FW_VERSION,
               app->id.c_str(), net::ipString().c_str());
  }
}

static void readLines(Stream& in, String& buf) {
  while (in.available()) {
    const char ch = in.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length()) handleLine(buf);
      buf = "";
    } else if (ch >= 32 && buf.length() < 120) {
      buf += ch;
    }
  }
}

// ================================================================ OTA (ArduinoOTA, cổng 3232)
static void otaBegin(const char* id) {
  ArduinoOTA.setHostname((String("tworobot-") + id).c_str());
  ArduinoOTA.setPassword("tworobot");
  ArduinoOTA.setMdnsEnabled(false);  // mDNS đã bật ở net::begin
  ArduinoOTA.onStart([]() {
    // An toàn: dừng mọi chuyển động trước khi ghi flash
    xSemaphoreTake(appMutex, portMAX_DELAY);
    JsonDocument cmd, out;
    cmd["op"] = "estop";
    app->handleCmd(cmd.as<JsonObjectConst>(), out.to<JsonObject>());
    xSemaphoreGive(appMutex);
    hal.setMotor('l', 0, true);
    hal.setMotor('r', 0, true);
    con.println("OTA: bắt đầu nạp — motor đã E-STOP");
  });
  ArduinoOTA.onEnd([]() { con.println("OTA: xong, khởi động lại"); });
  ArduinoOTA.onError([](ota_error_t e) { con.printf("OTA LỖI %u\n", (unsigned)e); });
  ArduinoOTA.begin();
}

void loop() {
  static String serialBuf, telnetBuf;
  static bool netReady = false;
  if (!netReady) {
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    otaBegin(app->id.c_str());
    netReady = true;
  }
  ArduinoOTA.handle();
  // telnet: nhận client mới (thay client cũ)
  if (telnetServer.hasClient()) {
    if (telnet && telnet.connected()) telnet.stop();
    telnet = telnetServer.available();
    telnet.printf("two-robot %s · id %s — gõ help\r\n", tr::FW_VERSION, app->id.c_str());
  }
  readLines(Serial, serialBuf);
  if (telnet && telnet.connected()) readLines(telnet, telnetBuf);
  // gửi log đang chờ ra telnet (không chặn task khác)
  uint8_t chunk[256];
  size_t n;
  while ((n = con.take(chunk, sizeof(chunk))) > 0) {
    if (telnet && telnet.connected()) telnet.write(chunk, n);
  }
  static uint32_t lastWatch = 0;
  if (watchOn && millis() - lastWatch > 500) {
    lastWatch = millis();
    printStatus();
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

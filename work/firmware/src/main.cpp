// two-robot firmware ESP32 — xem work/firmware/README.md
//
// Task (FreeRTOS):
//   control  core 1, 100 Hz, ưu tiên 5: encoder, gyro, ước lượng, primitive, PWM, an toàn
//   range    core 1, ~30 Hz, ưu tiên 3: đọc 2 ToF (chặn ~33 ms) → hộp thư cho control
//   http     core 0, ưu tiên 2:        WebServer (route /api/* → App, khoá chung mutex)
#include <Arduino.h>

#include "app.h"
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

static void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  uint32_t lastSeq = 0, maxUs = 0, lastLog = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    const uint32_t t0 = micros();
    TofSample s;
    portENTER_CRITICAL(&tofMux);
    s = tofBox;
    portEXIT_CRITICAL(&tofMux);
    xSemaphoreTake(appMutex, portMAX_DELAY);
    if (s.seq != lastSeq) {
      lastSeq = s.seq;
      app->onRange(s.l, s.ls, s.r, s.rs);
    }
    app->tick(0.01f);
    const bool led = app->ledOn(millis());
    xSemaphoreGive(appMutex);
    hal.setLed(led);
    const uint32_t us = micros() - t0;
    if (us > maxUs) maxUs = us;
    if (millis() - lastLog > 10000) {  // nhật ký nhẹ mỗi 10 s
      lastLog = millis();
      Serial.printf("[%lus] state=%s role=%s ip=%s tick_max=%luus heap=%u hw=0x%x\n", millis() / 1000,
                    app->state.c_str(), app->role.c_str(), net::ipString().c_str(), (unsigned long)maxUs,
                    ESP.getFreeHeap(), (unsigned)hal.hwFault());
      maxUs = 0;
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
  String name = hal.prefs.getString("name", String("Robot-") + id);

  app = new tr::App(hal, id, name.c_str());
  app->begin();
  appMutex = xSemaphoreCreateMutex();
  Serial.printf("id=%s name=%s role=%s cfg_rev=%u%s\n", id, name.c_str(), app->role.c_str(), app->params.rev,
                app->params.flag_default ? " (CẤU HÌNH MẶC ĐỊNH — cần Save trên Web Control trước khi chạy)" : "");

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(rangeTask, "range", 4096, nullptr, 3, nullptr, 1);

  net::begin(hal.prefs, id);
  net::startHttp(app, appMutex, &hal.prefs);
  xTaskCreatePinnedToCore(net::httpTask, "http", 12288, nullptr, 2, nullptr, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }

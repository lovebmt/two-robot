#pragma once
// WiFi (STA, hoặc AP để nhập WiFi) + HTTP server theo wiki/protocol.md (chỉ HTTP, cổng 80).
#include <Arduino.h>
#include <Preferences.h>

#include "app.h"

namespace net {

// Kết nối WiFi đã lưu (NVS "ssid"/"pass", hoặc secrets.h). Không được trong 15 s → phát AP
// "TwoRobot-<id>" (mật khẩu "tworobot"), vào http://192.168.4.1 để nhập WiFi.
bool begin(Preferences& prefs, const String& id);
bool isAp();
String ipString();

// HTTP: mọi /api/* chuyển cho App trong mutex chung với task điều khiển.
void startHttp(tr::App* app, SemaphoreHandle_t mutex, Preferences* prefs);
void httpTask(void*);

}  // namespace net

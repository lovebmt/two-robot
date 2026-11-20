#pragma once
// WiFi (STA, hoặc AP để nhập WiFi) + HTTP server theo wiki/protocol.md (chỉ HTTP, cổng 80).
#include <Arduino.h>
#include <Preferences.h>

#include <deque>
#include <string>

#include "app.h"

// Định nghĩa trong main.cpp: sao chép snapshot telemetry + sự kiện (không khoá điều khiển)
void stateSnapshot(std::string& tel, std::deque<tr::App::Ev>& ev, uint32_t& eseq);

namespace net {

// Kết nối WiFi đã lưu (NVS "ssid"/"pass", hoặc secrets.h). Không được trong 15 s → phát AP
// "TwoRobot-<id>" (mật khẩu "tworobot"), vào http://192.168.4.1 để nhập WiFi.
bool begin(Preferences& prefs, const String& id);
bool isAp();
String ipString();

// Chẩn đoán: khoảng lặng dài nhất giữa 2 request từ PC (ms, đọc rồi xoá), số lần WiFi rớt, lý do lần rớt cuối
uint32_t takeGapMax();
uint32_t disconnects();
uint32_t lastDisconnectReason();

// HTTP: mọi /api/* chuyển cho App trong mutex chung với task điều khiển.
void startHttp(tr::App* app, SemaphoreHandle_t mutex, Preferences* prefs);
void httpTask(void*);

}  // namespace net

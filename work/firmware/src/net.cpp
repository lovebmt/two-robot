#include "net.h"

#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>

#if __has_include("secrets.h")
#include "secrets.h"  // tuỳ chọn: #define WIFI_SSID "..." / #define WIFI_PASS "..." (không commit)
#endif

namespace net {

static WebServer server(80);
static tr::App* g_app = nullptr;
static SemaphoreHandle_t g_mtx = nullptr;
static Preferences* g_prefs = nullptr;
static bool g_ap = false;

bool isAp() { return g_ap; }
String ipString() { return g_ap ? WiFi.softAPIP().toString() : WiFi.localIP().toString(); }

bool begin(Preferences& prefs, const String& id) {
  String ssid = prefs.isKey("ssid") ? prefs.getString("ssid", "") : String();
  String pass = prefs.isKey("pass") ? prefs.getString("pass", "") : String();
#ifdef WIFI_SSID
  if (ssid.isEmpty()) { ssid = WIFI_SSID; pass = WIFI_PASS; }
#endif
  const String host = "tworobot-" + id;
  if (!ssid.isEmpty()) {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(host.c_str());
    WiFi.setSleep(false);  // bắt buộc: power-save làm trễ 100–300 ms
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.printf("WiFi: kết nối \"%s\"", ssid.c_str());
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; ++i) { delay(500); Serial.print('.'); }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      g_ap = false;
      if (MDNS.begin(host.c_str())) MDNS.addService("http", "tcp", 80);
      Serial.printf("WiFi OK: http://%s  (http://%s.local)\n", WiFi.localIP().toString().c_str(), host.c_str());
      return true;
    }
    Serial.println("WiFi: không kết nối được → chuyển AP");
  }
  WiFi.mode(WIFI_AP);
  const String ap = "TwoRobot-" + id;
  WiFi.softAP(ap.c_str(), "tworobot");
  g_ap = true;
  Serial.printf("AP \"%s\" (mật khẩu tworobot): vào http://%s để nhập WiFi\n", ap.c_str(),
                WiFi.softAPIP().toString().c_str());
  return false;
}

// RAII: khoá App chung với task điều khiển (giữ ngắn: handler chỉ đọc/ghi trạng thái)
struct Lock {
  bool ok;
  explicit Lock(TickType_t to) : ok(xSemaphoreTake(g_mtx, to) == pdTRUE) {}
  ~Lock() { if (ok) xSemaphoreGive(g_mtx); }
};

static void sendJson(int code, JsonDocument& doc) {
  String s;
  serializeJson(doc, s);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", s);
}

static const char* methodName(HTTPMethod m) {
  switch (m) {
    case HTTP_GET: return "GET";
    case HTTP_PUT: return "PUT";
    case HTTP_POST: return "POST";
    case HTTP_DELETE: return "DELETE";
    default: return "OTHER";
  }
}

static void handleApi() {
  const String uri = server.uri();
  if (!uri.startsWith("/api/")) {
    server.send(404, "text/plain", "not found");
    return;
  }
  JsonDocument body;
  const HTTPMethod m = server.method();
  if (m == HTTP_GET) {
    for (int i = 0; i < server.args(); ++i) {
      const String& v = server.arg(i);
      char* end = nullptr;
      const long n = strtol(v.c_str(), &end, 10);
      if (end && *end == 0 && v.length()) body[server.argName(i).c_str()] = n;
      else body[server.argName(i).c_str()] = v.c_str();
    }
  } else if (server.hasArg("plain") && server.arg("plain").length()) {
    DeserializationError e = deserializeJson(body, server.arg("plain"));
    if (e) {
      JsonDocument out;
      out["ok"] = false;
      out["err"] = String("JSON lỗi: ") + e.c_str();
      sendJson(400, out);
      return;
    }
  }
  JsonDocument out;
  int code;
  {
    Lock lk(pdMS_TO_TICKS(50));
    if (!lk.ok) {
      out["ok"] = false;
      out["err"] = "busy";
      sendJson(503, out);
      return;
    }
    code = g_app->handleRequest(methodName(m), std::string(uri.c_str()), body.as<JsonObjectConst>(), out);
  }
  sendJson(code, out);
}

static void handleRoot() {
  String ip = ipString();
  String html =
      "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>"
      "<title>TwoRobot</title><body style='font-family:system-ui;max-width:520px;margin:24px auto;padding:0 16px'>"
      "<h2>TwoRobot ";
  html += g_app ? g_app->name.c_str() : "";
  html += "</h2><p>ID <b>";
  html += g_app ? g_app->id.c_str() : "";
  html += "</b> · fw ";
  html += tr::FW_VERSION;
  html += "<br>Chế độ: <b>";
  html += g_ap ? "AP (chưa có WiFi)" : "WiFi";
  html += "</b> · IP <b>" + ip + "</b></p>";
  html += "<p>Web Control (PC): tab Thiết bị → Quét <code>" + ip + "</code></p>";
  html +=
      "<h3>Cài WiFi</h3><form method=post action=/wifi>"
      "<p>SSID<br><input name=ssid style='width:100%'></p>"
      "<p>Mật khẩu<br><input name=pass type=password style='width:100%'></p>"
      "<p><button>Lưu và khởi động lại</button></p></form>"
      "<p><a href=/api/info>/api/info</a> · <a href='/api/state?since=0'>/api/state</a></p>";
  server.send(200, "text/html; charset=utf-8", html);
}

static void handleWifi() {
  const String ssid = server.arg("ssid");
  if (ssid.isEmpty()) {
    server.send(400, "text/plain; charset=utf-8", "Thiếu SSID");
    return;
  }
  g_prefs->putString("ssid", ssid);
  g_prefs->putString("pass", server.arg("pass"));
  server.send(200, "text/html; charset=utf-8",
              "<meta charset=utf-8>Đã lưu WiFi. Robot khởi động lại sau 1 s, rồi tìm robot bằng nút Quét trên Web Control.");
  delay(1000);
  ESP.restart();
}

void startHttp(tr::App* app, SemaphoreHandle_t mutex, Preferences* prefs) {
  g_app = app;
  g_mtx = mutex;
  g_prefs = prefs;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/wifi", HTTP_POST, handleWifi);
  server.onNotFound(handleApi);  // mọi /api/* (route chi tiết nằm trong App::handleRequest)
  server.begin();
}

void httpTask(void*) {
  for (;;) {
    server.handleClient();
    vTaskDelay(1);
  }
}

}  // namespace net

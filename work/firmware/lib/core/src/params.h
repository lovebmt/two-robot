#pragma once
// Tham số động: truy cập theo khoá, validate, ràng buộc chéo, giá trị suy ra.
// Tương ứng tworobot/params.py. Struct Config sinh từ schema (config_gen.h).
#include <ArduinoJson.h>

#include <map>
#include <string>
#include <vector>

#include "config_gen.h"

namespace tr {

struct Derived {
  float ticks_per_rev = 0;
  float mm_per_tick_l = 0;
  float mm_per_tick_r = 0;
  float r_sweep_mm = 0;
  float v_wheel_max_mm_s = 0;
};

using Errors = std::map<std::string, std::string>;

const ParamMeta* findParam(const char* key);
float getNum(const Config& c, const ParamMeta& m);
bool getBool(const Config& c, const ParamMeta& m);

// Áp patch JSON {key: value} lên bản copy. Trả false + errors nếu có lỗi (không dùng `out`).
bool validatePatch(const Config& base, JsonObjectConst patch, Config& out, Errors& errs);
void crossCheck(const Config& c, Errors& errs);

// Bitmask nhóm thay đổi giữa a và b; idleKeys nhận các khoá 'idle' đã đổi.
uint32_t changedGroups(const Config& a, const Config& b, std::vector<std::string>* idleKeys = nullptr);

float sweepRadius(const Config& c);
float wheelVmax(const Config& c, float vbat = 11.1f);
Derived derive(const Config& c);

void configToJson(const Config& c, JsonObject out);
void derivedToJson(const Derived& d, JsonObject out);
void groupsToJson(uint32_t mask, JsonArray out);
uint32_t crc32(const uint8_t* data, size_t len);
std::string configCrc(const Config& c);

}  // namespace tr

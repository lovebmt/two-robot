#include "hal_esp32.h"

#include <LittleFS.h>
#include <driver/pcnt.h>

#include "board/pins.h"

// ================================================================ MPU6050 (thanh ghi trực tiếp)
bool Mpu6050::write_(uint8_t reg, uint8_t val) {
  w_->beginTransmission(addr_);
  w_->write(reg);
  w_->write(val);
  return w_->endTransmission() == 0;
}

bool Mpu6050::begin(TwoWire& w, uint8_t addr) {
  w_ = &w;
  addr_ = addr;
  if (!write_(0x6B, 0x80)) { ok_ = false; return false; }  // PWR_MGMT_1: reset
  delay(100);
  ok_ = write_(0x6B, 0x01)       // clock = PLL gyro X
        && write_(0x1A, 0x03)    // CONFIG: DLPF ~42 Hz
        && write_(0x1B, 0x00)    // GYRO_CONFIG: ±250 °/s (131 LSB/°/s)
        && write_(0x19, 0x04);   // SMPLRT_DIV: 1 kHz / 5 = 200 Hz
  w_->beginTransmission(addr_);
  w_->write(0x75);  // WHO_AM_I
  if (w_->endTransmission(false) == 0 && w_->requestFrom(addr_, (uint8_t)1) == 1) {
    uint8_t who = w_->read();
    log_i("MPU6050 WHO_AM_I=0x%02x", who);  // bản clone có thể khác 0x68 (0x70/0x72/0x98)
  }
  return ok_ && read();
}

bool Mpu6050::read() {
  if (!w_) return false;
  w_->beginTransmission(addr_);
  w_->write(0x41);  // TEMP_OUT_H … GYRO_ZOUT_L (8 byte)
  if (w_->endTransmission(false) != 0 || w_->requestFrom(addr_, (uint8_t)8) != 8) {
    if (++fails_ > 20) ok_ = false;  // ~0.2 s liên tục lỗi → báo HW_IMU
    return false;
  }
  uint8_t b[8];
  for (int i = 0; i < 8; ++i) b[i] = w_->read();
  const int16_t t = (int16_t)((b[0] << 8) | b[1]);
  const int16_t z = (int16_t)((b[6] << 8) | b[7]);
  temp_ = t / 340.0f + 36.53f;
  gz_ = (z / 131.0f) * 0.017453292f;
  fails_ = 0;
  ok_ = true;
  return true;
}

// ================================================================ 2× VL53L0X
static bool initTof(VL53L0X& s, TwoWire& w, uint8_t xshut, uint8_t addr) {
  digitalWrite(xshut, HIGH);
  delay(10);
  s.setBus(&w);
  s.setTimeout(100);
  if (!s.init()) return false;
  if (addr != 0x29) s.setAddress(addr);
  s.setMeasurementTimingBudget(33000);  // ~30 Hz
  s.startContinuous(0);
  return true;
}

void TofPair::begin(TwoWire& w) {
  pinMode(pins::TOF_L_XSHUT, OUTPUT);
  pinMode(pins::TOF_R_XSHUT, OUTPUT);
  digitalWrite(pins::TOF_L_XSHUT, LOW);  // tắt cả 2 rồi bật lần lượt để đổi địa chỉ
  digitalWrite(pins::TOF_R_XSHUT, LOW);
  delay(10);
  ok_l_ = initTof(l_, w, pins::TOF_L_XSHUT, pins::TOF_L_ADDR);
  ok_r_ = initTof(r_, w, pins::TOF_R_XSHUT, pins::TOF_R_ADDR);
}

// Đọc 1 mẫu continuous kèm range status + cường độ tín hiệu (thư viện Pololu chỉ trả mm, không lọc gì →
// khi phía trước trống chip vẫn trả "số ma" vài chục cm với tín hiệu yếu / status không hợp lệ).
// Bố cục thanh ghi theo ST API (VL53L0X_GetRangingMeasurementData): 12 byte từ RESULT_RANGE_STATUS 0x14.
bool TofPair::readOne_(VL53L0X& s, Raw& out) {
  const uint32_t t0 = millis();
  while ((s.readReg(0x13) & 0x07) == 0) {  // RESULT_INTERRUPT_STATUS: chờ có mẫu mới
    if (millis() - t0 > 100) return false;
    delay(1);
  }
  uint8_t b[12];
  s.readMulti(0x14, b, 12);
  s.writeReg(0x0B, 0x01);  // SYSTEM_INTERRUPT_CLEAR
  out.dev_status = (b[0] >> 3) & 0x0F;
  out.signal = ((b[6] << 8) | b[7]) / 128.f;   // fix 9.7 MCPS
  out.ambient = ((b[8] << 8) | b[9]) / 128.f;
  out.mm = (b[10] << 8) | b[11];
  return true;
}

void TofPair::classify_(const Raw& r, bool timeout, float& mm, int& st) const {
  mm = 8190;
  st = 4;
  if (timeout || r.mm >= 8190) return;
  if (r.dev_status != 11 || r.signal < min_signal_mcps) { st = 2; return; }  // 11 = range valid
  mm = r.mm;
  st = 0;
}

void TofPair::read(float& l_mm, int& l_st, float& r_mm, int& r_st) {
  l_mm = r_mm = 8190;
  l_st = r_st = 4;
  if (ok_l_) {
    const bool ok = readOne_(l_, raw_l_);
    fail_l_ = ok ? 0 : fail_l_ + 1;
    classify_(raw_l_, !ok, l_mm, l_st);
  }
  if (ok_r_) {
    const bool ok = readOne_(r_, raw_r_);
    fail_r_ = ok ? 0 : fail_r_ + 1;
    classify_(raw_r_, !ok, r_mm, r_st);
  }
  if (fail_l_ > 10) ok_l_ = false;  // timeout liên tục → báo HW_TOF_L
  if (fail_r_ > 10) ok_r_ = false;
  if (!ok_l_ && !ok_r_) delay(33);
}

// ================================================================ Esp32Hal
static void scanBus(TwoWire& w, const char* label, const char* need) {
  Serial.printf("I2C %s:", label);
  int n = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    w.beginTransmission(a);
    if (w.endTransmission() == 0) {
      Serial.printf(" 0x%02x", a);
      ++n;
    }
  }
  Serial.printf("  → %d thiết bị (cần: %s)\n", n, need);
}

void Esp32Hal::i2cScan() {
  scanBus(Wire, "MPU (SDA 21, SCL 22)", "0x68");
  scanBus(Wire1, "ToF (SDA 25, SCL 26)", "0x29 ToF phải, 0x30 ToF trái");
  Serial.printf("ToF trái %s · ToF phải %s · MPU6050 %s\n", tof.okL() ? "OK" : "LỖI", tof.okR() ? "OK" : "LỖI",
                imu.ok() ? "OK" : "LỖI");
}

static constexpr int16_t PCNT_LIM = 30000;  // PCNT tự về 0 khi chạm ±LIM → xử lý quấn trong readEncoder

static void setupPcnt(pcnt_unit_t unit, uint8_t a, uint8_t b) {
  pcnt_config_t c = {};
  c.unit = unit;
  c.counter_h_lim = PCNT_LIM;
  c.counter_l_lim = -PCNT_LIM;
  // Kênh 0: xung A, điều khiển B; kênh 1: xung B, điều khiển A → đếm x4
  c.channel = PCNT_CHANNEL_0;
  c.pulse_gpio_num = a;
  c.ctrl_gpio_num = b;
  c.pos_mode = PCNT_COUNT_DEC;
  c.neg_mode = PCNT_COUNT_INC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_unit_config(&c);
  c.channel = PCNT_CHANNEL_1;
  c.pulse_gpio_num = b;
  c.ctrl_gpio_num = a;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(unit, 250);  // bỏ gai < ~3 µs
  pcnt_filter_enable(unit);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void Esp32Hal::begin() {
  // Motor: tắt trước tiên (an toàn khi boot)
  const uint8_t outs[] = {pins::MOTOR_L_IN1, pins::MOTOR_L_IN2, pins::MOTOR_R_IN1, pins::MOTOR_R_IN2};
  for (uint8_t p : outs) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  ledcSetup(pins::PWM_CH_L, 2000, pins::PWM_BITS);
  ledcSetup(pins::PWM_CH_R, 2000, pins::PWM_BITS);
  ledcAttachPin(pins::MOTOR_L_EN, pins::PWM_CH_L);
  ledcAttachPin(pins::MOTOR_R_EN, pins::PWM_CH_R);
  ledcWrite(pins::PWM_CH_L, 0);
  ledcWrite(pins::PWM_CH_R, 0);
  if (pins::LED != 255) pinMode(pins::LED, OUTPUT);

  setupPcnt(PCNT_UNIT_0, pins::ENC_L_A, pins::ENC_L_B);
  setupPcnt(PCNT_UNIT_1, pins::ENC_R_A, pins::ENC_R_B);

  // 2 bus I2C riêng: Wire = MPU6050 (task control), Wire1 = 2 ToF (task range)
  Wire.begin(pins::I2C_IMU_SDA, pins::I2C_IMU_SCL, 400000);
  Wire1.begin(pins::I2C_TOF_SDA, pins::I2C_TOF_SCL, 400000);
  tof.begin(Wire1);  // bật lần lượt qua XSHUT, đổi địa chỉ ToF trái → 0x30
  if (!imu.begin(Wire, pins::MPU_ADDR)) Serial.println("LỖI: MPU6050 không phản hồi ở 0x68 (kiểm tra SDA 21 / SCL 22, AD0)");
  i2cScan();

  if (pins::VBAT_ADC != 255) analogSetPinAttenuation(pins::VBAT_ADC, ADC_11db);
  prefs.begin("tworobot", false);
  fs_ok_ = LittleFS.begin(true);
  if (fs_ok_ && !LittleFS.exists("/cfg")) LittleFS.mkdir("/cfg");
}

void Esp32Hal::setMotor(char side, float duty, bool brake) {
  const bool l = side == 'l';
  const uint8_t in1 = l ? pins::MOTOR_L_IN1 : pins::MOTOR_R_IN1;
  const uint8_t in2 = l ? pins::MOTOR_L_IN2 : pins::MOTOR_R_IN2;
  const uint8_t ch = l ? pins::PWM_CH_L : pins::PWM_CH_R;
  const uint32_t full = (1u << pins::PWM_BITS) - 1;
  if (brake) {  // phanh nhanh: IN1 = IN2 = HIGH, EN = 1
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    ledcWrite(ch, full);
    return;
  }
  if (duty == 0.f) {  // thả trôi
    ledcWrite(ch, 0);
    return;
  }
  digitalWrite(in1, duty > 0 ? HIGH : LOW);
  digitalWrite(in2, duty > 0 ? LOW : HIGH);
  float a = fabsf(duty);
  if (a > 1.f) a = 1.f;
  ledcWrite(ch, (uint32_t)(a * full + 0.5f));
}

int32_t Esp32Hal::readEncoder(char side) {
  // Gọi đều 100 Hz: giữa 2 lần đọc thay đổi << LIM/2 → phát hiện được lần PCNT tự về 0
  const int i = side == 'l' ? 0 : 1;
  int16_t c = 0;
  pcnt_get_counter_value(i == 0 ? PCNT_UNIT_0 : PCNT_UNIT_1, &c);
  int32_t d = (int32_t)c - last_cnt_[i];
  if (d > PCNT_LIM / 2) d -= PCNT_LIM;
  else if (d < -PCNT_LIM / 2) d += PCNT_LIM;
  last_cnt_[i] = c;
  total_[i] += d;
  return total_[i];
}

float Esp32Hal::readGyroZ() {
  imu.read();  // 1 burst I2C ~0.25 ms ở 400 kHz; cập nhật cả nhiệt độ
  return imu.gyroZ();
}

float Esp32Hal::readVbat(float divider) {
  if (divider <= 0 || pins::VBAT_ADC == 255) return NAN;  // không đo pin
  return analogReadMilliVolts(pins::VBAT_ADC) / 1000.0f * divider;
}

void Esp32Hal::setPwmFreq(int hz) {
  ledcChangeFrequency(pins::PWM_CH_L, hz, pins::PWM_BITS);
  ledcChangeFrequency(pins::PWM_CH_R, hz, pins::PWM_BITS);
}

void Esp32Hal::setLed(bool on) {
  if (pins::LED != 255) digitalWrite(pins::LED, on ? HIGH : LOW);
}

uint32_t Esp32Hal::hwFault() {
  uint32_t f = 0;
  if (!imu.ok()) f |= tr::HW_IMU;
  if (!tof.okL()) f |= tr::HW_TOF_L;
  if (!tof.okR()) f |= tr::HW_TOF_R;
  return f;
}

// "Flash": ghi atomic tmp → kiểm tra đọc lại → bak → đổi tên
bool Esp32Hal::storageLoad(std::string& json) {
  if (!fs_ok_) return false;
  for (const char* path : {"/cfg/config.json", "/cfg/config.bak"}) {
    if (!LittleFS.exists(path)) continue;
    File f = LittleFS.open(path, "r");
    if (!f) continue;
    json.clear();
    json.reserve(f.size());
    while (f.available()) json += (char)f.read();
    f.close();
    if (!json.empty() && json[0] == '{') return true;
  }
  return false;
}

bool Esp32Hal::storageSave(const std::string& json) {
  if (!fs_ok_) return false;
  File f = LittleFS.open("/cfg/config.tmp", "w");
  if (!f) return false;
  const size_t n = f.write((const uint8_t*)json.data(), json.size());
  f.close();
  if (n != json.size()) return false;
  if (LittleFS.exists("/cfg/config.bak")) LittleFS.remove("/cfg/config.bak");
  if (LittleFS.exists("/cfg/config.json")) LittleFS.rename("/cfg/config.json", "/cfg/config.bak");
  return LittleFS.rename("/cfg/config.tmp", "/cfg/config.json");
}

std::string Esp32Hal::nvsGetRole() {
  return prefs.isKey("role") ? std::string(prefs.getString("role", "").c_str()) : std::string();
}

void Esp32Hal::nvsSetRole(const std::string& role) { prefs.putString("role", role.c_str()); }

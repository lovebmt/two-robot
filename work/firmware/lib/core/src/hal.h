#pragma once
// Interface phần cứng — ESP32 (src/hal_esp32.*) và test native cùng cài đặt interface này.
// Tương ứng tworobot/fw/hal.py.
#include <stdint.h>
#include <string>

namespace tr {

// Bit lỗi phần cứng (Hal::hwFault)
constexpr uint32_t HW_IMU = 1u << 0;    // MPU6050 không phản hồi
constexpr uint32_t HW_TOF_L = 1u << 1;  // ToF trái không khởi tạo/đọc được
constexpr uint32_t HW_TOF_R = 1u << 2;

class Hal {
 public:
  virtual ~Hal() {}
  virtual uint32_t nowMs() = 0;
  // side 'l'|'r'; duty ∈ [-1, 1]; brake = phanh nhanh L298N (IN1 = IN2 = HIGH)
  virtual void setMotor(char side, float duty, bool brake) = 0;
  virtual int32_t readEncoder(char side) = 0;  // tổng tick có dấu (PCNT)
  virtual float readGyroZ() = 0;               // rad/s thô (chưa trừ bias)
  virtual float readImuTemp() = 0;             // °C nhiệt chip MPU6050
  // Điện áp pin (V). divider <= 0 → không đo được, trả NAN.
  virtual float readVbat(float divider) = 0;
  virtual void setPwmFreq(int hz) { (void)hz; }
  virtual uint32_t hwFault() { return 0; }  // bitmask HW_*
  virtual void setLed(bool on) { (void)on; }
  // "Flash": /cfg/config.json (LittleFS) và NVS (vai trò)
  virtual bool storageLoad(std::string& json) = 0;
  virtual bool storageSave(const std::string& json) = 0;
  virtual std::string nvsGetRole() = 0;
  virtual void nvsSetRole(const std::string& role) = 0;
};

}  // namespace tr

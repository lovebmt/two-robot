#pragma once
// HAL cho ESP32 + L298N + GA25-370 (PCNT) + MPU6050 + 2× VL53L0X + LittleFS/NVS.
#include <Arduino.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "hal.h"

class Mpu6050 {
 public:
  bool begin(TwoWire& w, uint8_t addr);
  bool read();  // đọc 1 lần TEMP + GYRO (8 byte), cập nhật cache
  float gyroZ() const { return gz_; }  // rad/s
  float tempC() const { return temp_; }
  bool ok() const { return ok_; }

 private:
  TwoWire* w_ = nullptr;
  uint8_t addr_ = 0x68;
  bool ok_ = false;
  int fails_ = 0;
  float gz_ = 0, temp_ = 25;
  bool write_(uint8_t reg, uint8_t val);
};

class TofPair {
 public:
  void begin(TwoWire& w);
  // Chặn tới khi có mẫu (~33 ms). status: 0 hợp lệ, 4 ngoài tầm / timeout.
  void read(float& l_mm, int& l_st, float& r_mm, int& r_st);
  bool okL() const { return ok_l_; }
  bool okR() const { return ok_r_; }

 private:
  VL53L0X l_, r_;
  bool ok_l_ = false, ok_r_ = false;
  int fail_l_ = 0, fail_r_ = 0;
};

class Esp32Hal : public tr::Hal {
 public:
  void begin();
  TofPair tof;
  Mpu6050 imu;
  Preferences prefs;

  uint32_t nowMs() override { return millis(); }
  void setMotor(char side, float duty, bool brake) override;
  int32_t readEncoder(char side) override;
  float readGyroZ() override;
  float readImuTemp() override { return imu.tempC(); }
  float readVbat(float divider) override;
  void setPwmFreq(int hz) override;
  void setLed(bool on) override;
  uint32_t hwFault() override;
  bool storageLoad(std::string& json) override;
  bool storageSave(const std::string& json) override;
  std::string nvsGetRole() override;
  void nvsSetRole(const std::string& role) override;

 private:
  int16_t last_cnt_[2] = {0, 0};
  int32_t total_[2] = {0, 0};
  bool fs_ok_ = false;
};

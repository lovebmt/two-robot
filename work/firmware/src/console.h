#pragma once
// Console: in ra Serial (USB) + đẩy vào bộ đệm vòng cho telnet (WiFi, cổng 23).
// Ghi KHÔNG chặn (task control 100 Hz có thể gọi); loop() gửi dần ra telnet.
#include <Arduino.h>

class Console : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* buf, size_t n) override;
  // Lấy tối đa `max` byte đang chờ gửi (gọi từ loop)
  size_t take(uint8_t* out, size_t max);

 private:
  static constexpr size_t CAP = 4096;
  uint8_t ring_[CAP];
  size_t head_ = 0, tail_ = 0;  // head: ghi, tail: đọc
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

extern Console con;

#include "console.h"

Console con;

size_t Console::write(const uint8_t* buf, size_t n) {
  Serial.write(buf, n);
  portENTER_CRITICAL(&mux_);
  for (size_t i = 0; i < n; ++i) {
    const size_t nx = (head_ + 1) % CAP;
    if (nx == tail_) tail_ = (tail_ + 1) % CAP;  // đầy: bỏ byte cũ nhất (không chặn)
    ring_[head_] = buf[i];
    head_ = nx;
  }
  portEXIT_CRITICAL(&mux_);
  return n;
}

size_t Console::take(uint8_t* out, size_t max) {
  size_t n = 0;
  portENTER_CRITICAL(&mux_);
  while (n < max && tail_ != head_) {
    out[n++] = ring_[tail_];
    tail_ = (tail_ + 1) % CAP;
  }
  portEXIT_CRITICAL(&mux_);
  return n;
}

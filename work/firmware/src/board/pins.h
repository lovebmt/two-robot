#pragma once
// GPIO CỐ ĐỊNH TRONG CODE (D19). Theo bảng đề xuất work/hardware/hardware-intake.md §3 —
// ⚠ KIỂM TRA LẠI CHO KHỚP DÂY THẬT TRƯỚC KHI NẠP. Sai chân có thể làm motor quay khi boot.
//
// Ràng buộc ESP32 (architecture.md §2.4): tránh 6–11 (flash); 34–39 chỉ input, KHÔNG có pull-up
// nội (encoder Hall open-collector cần điện trở kéo lên 3.3 V ngoài); ADC2 không dùng được khi bật WiFi.
#include <stdint.h>

namespace pins {

// L298N
constexpr uint8_t MOTOR_L_EN = 25;  // ENA (PWM) — tháo jumper ENA
constexpr uint8_t MOTOR_L_IN1 = 26;
constexpr uint8_t MOTOR_L_IN2 = 27;
constexpr uint8_t MOTOR_R_EN = 14;  // ENB (PWM) — tháo jumper ENB
constexpr uint8_t MOTOR_R_IN1 = 23;  // IN3
constexpr uint8_t MOTOR_R_IN2 = 5;   // IN4 (strapping, dùng làm output sau boot được)

// Encoder GA25-370 (2 kênh Hall). Đếm bằng PCNT, x4.
constexpr uint8_t ENC_L_A = 34;
constexpr uint8_t ENC_L_B = 35;
constexpr uint8_t ENC_R_A = 36;  // SVP
constexpr uint8_t ENC_R_B = 39;  // SVN

// I2C0: 2× ToF200C (VL53L0X)
constexpr uint8_t I2C0_SDA = 21;
constexpr uint8_t I2C0_SCL = 22;
constexpr uint8_t TOF_L_XSHUT = 18;
constexpr uint8_t TOF_R_XSHUT = 19;
constexpr uint8_t TOF_L_ADDR = 0x30;  // đổi địa chỉ lúc boot qua XSHUT
constexpr uint8_t TOF_R_ADDR = 0x29;  // mặc định

// I2C1: MPU6050 (bus riêng để đọc gyro 100 Hz không bị ToF chặn)
constexpr uint8_t I2C1_SDA = 13;
constexpr uint8_t I2C1_SCL = 4;
constexpr uint8_t MPU_ADDR = 0x68;  // AD0 = GND

// Khác
constexpr uint8_t VBAT_ADC = 32;  // ADC1, qua cầu chia (hệ số: power.vbat_divider)
constexpr uint8_t LED = 2;        // LED onboard DevKit

// LEDC
constexpr uint8_t PWM_CH_L = 0;
constexpr uint8_t PWM_CH_R = 1;
constexpr uint8_t PWM_BITS = 10;

static_assert(MOTOR_L_EN < 34 && MOTOR_L_IN1 < 34 && MOTOR_L_IN2 < 34 && MOTOR_R_EN < 34 && MOTOR_R_IN1 < 34 &&
                  MOTOR_R_IN2 < 34 && TOF_L_XSHUT < 34 && TOF_R_XSHUT < 34 && LED < 34,
              "Chân output không được là 34–39 (input-only)");
static_assert(!(MOTOR_L_EN >= 6 && MOTOR_L_EN <= 11) && !(MOTOR_R_EN >= 6 && MOTOR_R_EN <= 11),
              "Chân 6–11 nối flash");
static_assert(VBAT_ADC >= 32 && VBAT_ADC <= 39, "Đo pin phải dùng ADC1 (32–39)");

}  // namespace pins

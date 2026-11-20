#pragma once
// GPIO CỐ ĐỊNH TRONG CODE (D19). Bảng chân theo dây thật của người dùng (2026-11-20), đã sửa 2 chỗ:
//   • 2 ToF KHÔNG dùng 34/35 cho I2C (34–39 chỉ là input, không kéo SDA/SCL xuống được)
//     → 2 ToF dùng CHUNG bus I2C 21/22 với MPU6050 (địa chỉ 0x68 / 0x29 / 0x30, Wire có mutex).
//   • Encoder trái KHÔNG dùng GPIO12 (strapping MTDI: HIGH lúc reset → flash 1.8 V → không boot,
//     đã thấy log boot:0x33 + "invalid header") → chuyển sang GPIO35.
// ⚠ Encoder: cấp VCC 3.3 V (không 5 V). GPIO35 không có pull-up nội: encoder phải có pull-up
//   trên board hoặc thêm 10 kΩ lên 3.3 V.
#include <stdint.h>

namespace pins {

// L298N (chọn từ chân output an toàn, không strapping, không lúc boot phát xung)
constexpr uint8_t MOTOR_L_EN = 25;   // ENA (PWM) — tháo jumper ENA
constexpr uint8_t MOTOR_L_IN1 = 26;  // IN1
constexpr uint8_t MOTOR_L_IN2 = 16;  // IN2
constexpr uint8_t MOTOR_R_EN = 17;   // ENB (PWM) — tháo jumper ENB
constexpr uint8_t MOTOR_R_IN1 = 18;  // IN3
constexpr uint8_t MOTOR_R_IN2 = 19;  // IN4

// Encoder GA25-370 (đếm PCNT x4). Thứ tự A/B sai → bật enc.invert_* trên web.
constexpr uint8_t ENC_L_A = 13;
constexpr uint8_t ENC_L_B = 35;  // (trước là 12 — strapping, KHÔNG dùng)
constexpr uint8_t ENC_R_A = 14;
constexpr uint8_t ENC_R_B = 27;

// I2C chung: MPU6050 + 2× ToF200C (VL53L0X)
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t TOF_L_XSHUT = 33;
constexpr uint8_t TOF_R_XSHUT = 32;
constexpr uint8_t TOF_L_ADDR = 0x30;  // đổi địa chỉ lúc boot qua XSHUT
constexpr uint8_t TOF_R_ADDR = 0x29;  // mặc định
constexpr uint8_t MPU_ADDR = 0x68;    // AD0 = GND

// Khác
constexpr uint8_t VBAT_ADC = 34;  // ADC1 (input-only, hợp với đo áp) qua cầu chia — power.vbat_divider
constexpr uint8_t LED = 2;        // LED onboard DevKit

// LEDC
constexpr uint8_t PWM_CH_L = 0;
constexpr uint8_t PWM_CH_R = 1;
constexpr uint8_t PWM_BITS = 10;

// ---- kiểm tra lúc biên dịch ----
constexpr uint8_t OUTPUTS[] = {MOTOR_L_EN, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_R_EN, MOTOR_R_IN1, MOTOR_R_IN2,
                               TOF_L_XSHUT, TOF_R_XSHUT, LED, I2C_SDA, I2C_SCL};
constexpr uint8_t ALL[] = {MOTOR_L_EN, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_R_EN, MOTOR_R_IN1, MOTOR_R_IN2, ENC_L_A,
                           ENC_L_B, ENC_R_A, ENC_R_B, I2C_SDA, I2C_SCL, TOF_L_XSHUT, TOF_R_XSHUT, VBAT_ADC, LED};
constexpr bool noneInputOnly(int i = 0) {
  return i >= (int)sizeof(OUTPUTS) ? true : (OUTPUTS[i] < 34 && noneInputOnly(i + 1));
}
constexpr bool noneForbidden(int i = 0) {  // 6–11 flash, 12 strapping (flash voltage), 0/1/3 boot/UART
  return i >= (int)sizeof(ALL)
             ? true
             : (!(ALL[i] >= 6 && ALL[i] <= 12) && ALL[i] != 0 && ALL[i] != 1 && ALL[i] != 3 && noneForbidden(i + 1));
}
constexpr bool unique(int i = 0, int j = 1) {
  return i >= (int)sizeof(ALL) ? true
         : j >= (int)sizeof(ALL) ? unique(i + 1, i + 2)
                                 : (ALL[i] != ALL[j] && unique(i, j + 1));
}
static_assert(noneInputOnly(), "Chân output/I2C không được là 34–39 (input-only)");
static_assert(noneForbidden(), "Không dùng 6–12 (flash + strapping MTDI), 0, 1, 3");
static_assert(unique(), "Một chân bị gán 2 chức năng");
static_assert(VBAT_ADC >= 32 && VBAT_ADC <= 39, "Đo pin phải dùng ADC1 (32–39)");

}  // namespace pins

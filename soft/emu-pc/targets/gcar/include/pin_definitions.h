#include <Arduino.h>

const uint32_t MOTOR_LEFT_BWD = PA_3; /* Example PWM pin for motor left. D1 on salaea. */
const uint32_t MOTOR_LEFT_FWD = PA_2; /* Direction pin for motor left.  D2 on salaea. */
const uint32_t MOTOR_RIGHT_FWD = PA_0; /* PWM pin for motor right. D4 on salaea. */
const uint32_t MOTOR_RIGHT_BWD = PA_1; /* Direction pin for motor right. D5 on salaea. */

const uint32_t PIN_LED = PC_13;

// MPU6050 I2C address
const uint8_t MPU6050_ADDR = 0x68;

// Startup thresholds measured Jul 2026:
//   Left motor: 62% (PWM 158)
//   Right motor: 55% (PWM 140)

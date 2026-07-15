#include <Arduino.h>

const uint32_t MOTOR_LEFT_BWD = PA_0; /* Example PWM pin for motor left. D4 on salaea. */
const uint32_t MOTOR_LEFT_FWD = PA_1; /* Direction pin for motor left.  D5 on salaea. */
const uint32_t MOTOR_RIGHT_FWD = PA_3; /* PWM pin for motor right. D1 on salaea. */
const uint32_t MOTOR_RIGHT_BWD = PA_2; /* Direction pin for motor right. D2 on salaea. */

const uint32_t PIN_LED = PC_13;

// MPU6050 I2C address
const uint8_t MPU6050_ADDR = 0x68;

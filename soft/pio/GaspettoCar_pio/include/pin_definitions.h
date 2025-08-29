#include <Arduino.h>

// Define the GPIO pins we want to test
#define PIN_A_MOTOR_RIGHT PA_0 // Salaea D4
#define PIN_B_MOTOR_RIGHT PA_1 // Salaea D5
#define PIN_A_MOTOR_LEFT PA_2 // Salaea D2
#define PIN_B_MOTOR_LEFT PA_3 // Salaea D1
#define PIN_LED PC13

// nRF24L01 SPI pins (assuming using SPI1 on STM32)
#define NRF24_CE PB0
#define NRF24_CSN PB1

// MPU6050 I2C address
const uint8_t MPU6050_ADDR = 0x68;

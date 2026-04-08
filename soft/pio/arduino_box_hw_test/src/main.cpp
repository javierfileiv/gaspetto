#include "pin_definitions.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <stdint.h> /* For uint8_t type. */

/* nRF24L01 registers. */
#define NRF24_REG_CONFIG    0x00
#define NRF24_REG_STATUS    0x07
#define NRF24_CMD_READ_REG  0x00
#define NRF24_CMD_WRITE_REG 0x20

/* MPU6050 registers. */
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

void setup()
{
    uint8_t counter = 0;

    Serial.begin(115200);

    pinMode(PIN_MOSFET_5V_LEDS, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

    /* Step 3: Turn everything off at startup. */
    /* Reminder: P-channel MOSFET with pull-up uses active-low logic. */
    /* HIGH (or released in open-drain mode) = MOSFET OFF. */
    /* LOW (pulled to ground) = MOSFET ON. */

    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);     /* Turn off 5V rail. */
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH); /* Turn off 3.3V rail. */

    delay(2000);                                /* Wait 2 seconds before starting. */
    Serial.println("Starting power rail test...");
}

void loop()
{
    /* Step 1: Turn on the 3.3V sensor rail. */
    Serial.println("Turning sensors on (3V3_SWITCHED = ON)");
    digitalWrite(PIN_LED, LOW); /* Turn on onboard LED (active-low). */

    /* Turn on Q2 by pulling the gate to ground. */
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);

    delay(3000); /* Keep it on for 3 seconds (verify with multimeter). */

    /* Step 2: Turn off the 3.3V sensor rail. */
    Serial.println("Turning sensors off (3V3_SWITCHED = OFF)");
    digitalWrite(PIN_LED, HIGH); /* Turn off onboard LED. */

    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);

    delay(2000); /* Wait 2 seconds with all rails off. */

    /* Step 3: Turn on the 5V LED power rail. */
    Serial.println("Turning LED power on (5V_SWITCHED = ON)");
    digitalWrite(PIN_LED, LOW);

    /* Turn on Q1. */
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);

    delay(3000); /* Keep it on for 3 seconds (verify with multimeter). */

    /* Step 4: Turn off the 5V LED power rail. */
    Serial.println("Turning LED power off (5V_SWITCHED = OFF)");
    digitalWrite(PIN_LED, HIGH);

    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);

    delay(2000); /* Wait before restarting the cycle. */
}

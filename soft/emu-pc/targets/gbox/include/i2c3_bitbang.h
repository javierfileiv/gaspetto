#pragma once

#include "pin_definitions.h"

#include <Arduino.h>

/**
 * I2C3 Bit-Banging Implementation
 *
 * WHY THIS EXISTS:
 * The STM32F411CE's hardware I2C3 peripheral on PB4/PA8 pins has reliability issues.
 * Testing (Oct 2026) confirmed that while initialization and device detection work,
 * actual ADC conversions hang during reads. This bit-bang implementation provides
 * reliable I2C communication as a workaround.
 *
 * PERFORMANCE:
 * Uses ~10kHz I2C speed (50us delay between transitions). This is sufficient for
 * ADS1115 modules which support up to 400kHz.
 *
 * USAGE:
 * For PC emulation (non-ARDUINO), this provides the same API as hardware I2C.
 * For STM32 hardware (ARDUINO defined), use BitBangWire wrapper in GBox_pio to
 * integrate with Adafruit_ADS1X15 library which expects TwoWire*.
 *
 * TRADE-OFF:
 * BitBangWire inherits from TwoWire for API compatibility, wasting ~100 bytes RAM
 * (0.08% of 128KB) for an unused parent object. This is acceptable for maintaining
 * a single code path that works with the Adafruit library.
 *
 * TESTING:
 * See soft/pio/arduino_i2c3_bb_test for validation tests.
 */

/**
 * Initialize I2C3 bit-banging pins
 * Configures SDA and SCL pins as inputs with pull-ups
 */
void i2c3_bb_init();

/**
 * Generate I2C START condition
 * @return true if successful, false if SDA is stuck low
 */
bool i2c3_bb_start();

/**
 * Generate I2C STOP condition
 */
void i2c3_bb_stop();

/**
 * Write a byte to the I2C bus
 * @param data Byte to write
 * @return true if ACK received, false if NACK
 */
bool i2c3_bb_write_byte(uint8_t data);

/**
 * Read a byte from the I2C bus
 * @param ack true to send ACK, false to send NACK
 * @return Byte read from the bus
 */
uint8_t i2c3_bb_read_byte(bool ack);

/**
 * Probe for a device at the given address
 * @param address 7-bit I2C address
 * @return true if device responds with ACK, false otherwise
 */
bool i2c3_bb_probe(uint8_t address);

/**
 * Read a 16-bit register from a device
 * @param address 7-bit I2C address
 * @param reg Register address to read
 * @param value Output: 16-bit value read (high byte first)
 * @return true if successful, false on error
 */
bool i2c3_bb_read_register(uint8_t address, uint8_t reg, uint16_t &value);

/**
 * Write a 16-bit register to a device
 * @param address 7-bit I2C address
 * @param reg Register address to write
 * @param value 16-bit value to write (high byte first)
 * @return true if successful, false on error
 */
bool i2c3_bb_write_register(uint8_t address, uint8_t reg, uint16_t value);

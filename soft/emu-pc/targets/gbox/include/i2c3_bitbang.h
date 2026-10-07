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
 * Uses ~33kHz I2C speed (10us delay between transitions, 30us per bit). The bus rises
 * through the 4k7 pull-ups of the ADS1115 module, which leaves a wide margin at this rate,
 * and the modules support up to 400kHz anyway.
 *
 * USAGE:
 * Compiled only by the PlatformIO targets: INPUT_PULLUP, OUTPUT_OPEN_DRAIN and
 * delayMicroseconds have no PC counterpart. On STM32 builds the BitBangWire wrapper in
 * soft/pio/GBox_pio/include exposes these functions through the TwoWire interface that
 * Adafruit_ADS1X15 expects, which requires Arduino Core STM32 3.0.0 or later for the
 * dispatch to reach the bit-bang instead of the hardware peripheral.
 *
 * TRADE-OFF:
 * BitBangWire inherits from TwoWire because Adafruit_ADS1X15 takes a TwoWire* and nothing
 * else. The inherited state stays unused: its rx/tx buffers are allocated on demand and
 * never are, since every method that would touch them is replaced here. What that
 * inheritance really costs is flash, because a polymorphic base keeps the whole hardware
 * I2C path linked even though nothing ever calls it. Accepted for one code path shared by
 * both ADS buses.
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
 * Release I2C3 bit-banging pins
 * Puts SDA and SCL back in high-impedance so another master can own the bus
 */
void i2c3_bb_deinit();

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
 * Report whether SCL was free for the whole transaction
 *
 * A released SCL that stays low means another device owns the line, either clock stretching
 * this driver does not implement or a short. Bytes clocked after that are worthless, so a
 * caller able to report a failure must not report success. Cleared by i2c3_bb_start() and by
 * i2c3_bb_init(), so it covers one transaction at a time.
 *
 * @return false when a released SCL failed to rise during the current transaction
 */
bool i2c3_bb_bus_released();

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

/**
 * Report whether SCL rose every time the bus released it during the last transaction
 * @return true if the clock line was free, false if something held it low
 */
bool i2c3_bb_bus_released();

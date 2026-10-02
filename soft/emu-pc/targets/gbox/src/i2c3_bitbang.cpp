#include "i2c3_bitbang.h"

// ============================================================================
// I2C3 Bit-Banging Implementation
// ============================================================================

constexpr uint16_t kI2c3BitBangDelayUs = 50; // ~10kHz I2C speed

// Internal helper functions
static void i2c3_bb_setSDA(bool state)
{
    if (state) {
        pinMode(I2C3_SDA_PIN, INPUT_PULLUP);
    } else {
        pinMode(I2C3_SDA_PIN, OUTPUT);
        digitalWrite(I2C3_SDA_PIN, LOW);
    }
}

static void i2c3_bb_setSCL(bool state)
{
    if (state) {
        pinMode(I2C3_SCL_PIN, INPUT_PULLUP);
    } else {
        pinMode(I2C3_SCL_PIN, OUTPUT);
        digitalWrite(I2C3_SCL_PIN, LOW);
    }
}

static bool i2c3_bb_readSDA()
{
    return digitalRead(I2C3_SDA_PIN);
}

static void i2c3_bb_delay()
{
    delayMicroseconds(kI2c3BitBangDelayUs);
}

// Public API functions
void i2c3_bb_init()
{
    pinMode(I2C3_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C3_SCL_PIN, INPUT_PULLUP);
    i2c3_bb_setSDA(true);
    i2c3_bb_setSCL(true);
    i2c3_bb_delay();
}

bool i2c3_bb_start()
{
    i2c3_bb_setSDA(true);
    i2c3_bb_setSCL(true);
    i2c3_bb_delay();

    if (!i2c3_bb_readSDA()) {
        return false; // SDA stuck low
    }

    i2c3_bb_setSDA(false);
    i2c3_bb_delay();
    i2c3_bb_setSCL(false);
    i2c3_bb_delay();

    return true;
}

void i2c3_bb_stop()
{
    i2c3_bb_setSDA(false);
    i2c3_bb_setSCL(true);
    i2c3_bb_delay();
    i2c3_bb_setSDA(true);
    i2c3_bb_delay();
}

bool i2c3_bb_write_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--) {
        bool bit = (data >> i) & 0x01;
        i2c3_bb_setSDA(bit);
        i2c3_bb_delay();
        i2c3_bb_setSCL(true);
        i2c3_bb_delay();
        i2c3_bb_setSCL(false);
        i2c3_bb_delay();
    }

    i2c3_bb_setSDA(true);
    i2c3_bb_delay();
    i2c3_bb_setSCL(true);
    i2c3_bb_delay();

    bool ack = !i2c3_bb_readSDA();

    i2c3_bb_setSCL(false);
    i2c3_bb_delay();

    return ack;
}

uint8_t i2c3_bb_read_byte(bool ack)
{
    uint8_t data = 0;
    i2c3_bb_setSDA(true);

    for (int i = 7; i >= 0; i--) {
        i2c3_bb_setSCL(true);
        i2c3_bb_delay();
        if (i2c3_bb_readSDA()) {
            data |= (1 << i);
        }
        i2c3_bb_setSCL(false);
        i2c3_bb_delay();
    }

    i2c3_bb_setSDA(!ack);
    i2c3_bb_delay();
    i2c3_bb_setSCL(true);
    i2c3_bb_delay();
    i2c3_bb_setSCL(false);
    i2c3_bb_delay();

    return data;
}

bool i2c3_bb_probe(uint8_t address)
{
    if (!i2c3_bb_start()) {
        i2c3_bb_stop();
        return false;
    }
    bool ack = i2c3_bb_write_byte(address << 1);
    i2c3_bb_stop();
    return ack;
}

bool i2c3_bb_read_register(uint8_t address, uint8_t reg, uint16_t &value)
{
    if (!i2c3_bb_start()) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_write_byte(address << 1)) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_write_byte(reg)) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_start()) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_write_byte((address << 1) | 0x01)) {
        i2c3_bb_stop();
        return false;
    }

    uint8_t high = i2c3_bb_read_byte(true);
    uint8_t low = i2c3_bb_read_byte(false);
    i2c3_bb_stop();

    value = (high << 8) | low;
    return true;
}

bool i2c3_bb_write_register(uint8_t address, uint8_t reg, uint16_t value)
{
    if (!i2c3_bb_start()) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_write_byte(address << 1)) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_write_byte(reg)) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_write_byte((value >> 8) & 0xFF)) {
        i2c3_bb_stop();
        return false;
    }
    if (!i2c3_bb_write_byte(value & 0xFF)) {
        i2c3_bb_stop();
        return false;
    }
    i2c3_bb_stop();
    return true;
}

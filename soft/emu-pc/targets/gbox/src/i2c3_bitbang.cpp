#include "i2c3_bitbang.h"

// ============================================================================
// I2C3 Bit-Banging Implementation
// ============================================================================

// The line goes high passively: setSDA(true) and setSCL(true) only release the pin, so the
// rising edge is set by the 4k7 pull-ups of the ADS1115 module, not by this MCU. With the
// STM32 internal pull-up in parallel that is about 4k2, and even 200pF of bus rises in well
// under 2us, so 10us per transition leaves a wide margin. What actually bounds the speed
// here is the cost of switching each pin with pinMode(), not the analog behaviour of the bus.
constexpr uint16_t kI2c3BitBangDelayUs = 10; // ~25kHz I2C speed (40us per bit)

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

// Latched for the duration of one transaction and cleared by i2c3_bb_start(). Set when a
// released SCL fails to rise, which means something else owns the line: clock stretching we
// do not implement, or a short. Any byte clocked afterwards is worthless, so callers that can
// report a failure must not pretend the transfer succeeded.
static bool gI2c3SclHeld = false;

// Only SCL can be checked this way. On a read, SDA sitting low is a legitimate data bit, so
// its level carries no error information, whereas SCL is ours to release and must come up.
static void i2c3_bb_checkSCL()
{
    if (digitalRead(I2C3_SCL_PIN) == LOW) {
        gI2c3SclHeld = true;
    }
}

static void i2c3_bb_delay()
{
    delayMicroseconds(kI2c3BitBangDelayUs);
}

// Public API functions
void i2c3_bb_init()
{
    gI2c3SclHeld = false;
    i2c3_bb_setSDA(true);
    i2c3_bb_setSCL(true);
    i2c3_bb_delay();
}

void i2c3_bb_deinit()
{
    pinMode(I2C3_SDA_PIN, INPUT);
    pinMode(I2C3_SCL_PIN, INPUT);
}

bool i2c3_bb_start()
{
    gI2c3SclHeld = false;
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
        i2c3_bb_checkSCL();
        i2c3_bb_setSCL(false);
        i2c3_bb_delay();
    }

    i2c3_bb_setSDA(true);
    i2c3_bb_delay();
    i2c3_bb_setSCL(true);
    i2c3_bb_delay();
    i2c3_bb_checkSCL();

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
        i2c3_bb_checkSCL();
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
    bool held = gI2c3SclHeld;
    i2c3_bb_stop();

    if (held) {
        return false; // SCL never rose: the two bytes are meaningless
    }

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

bool i2c3_bb_bus_released()
{
    return !gI2c3SclHeld;
}

#pragma once

#include "i2c3_bitbang.h"

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

// Adafruit talks to the bus through a TwoWire*, so the bit-bang is reachable only if the
// methods below replace the hardware ones in the vtable. That needs the base to declare
// them virtual, which Arduino Core STM32 does from 3.0.0, where TwoWire derives from
// arduino::HardwareI2C. The override keywords are the guard: on an older core they report
// the dispatch bug as a compile error instead of letting a call bind statically to the
// hardware peripheral while write() and read() keep filling our own buffers.
class BitBangWire : public TwoWire {
public:
    BitBangWire(int sda, int scl)
            : TwoWire(sda, scl)
            , _txAddress(0)
            , _txBufferLength(0)
            , _txOverflow(false)
            , _transmitting(false)
            , _rxBufferIndex(0)
            , _rxBufferLength(0)
    {
    }

    void begin() override
    {
        i2c3_bb_init();
    }

    void begin(uint8_t address) override
    {
        (void)address;
        i2c3_bb_init();
    }

    void end() override
    {
        i2c3_bb_deinit();
    }

    void setClock(uint32_t frequency) override
    {
        (void)frequency;
    }

    // Meaningless here on purpose: the two lines are fixed by i2c3_bitbang.cpp and the rate is
    // set by its delay constant. Kept because GBox.cpp calls them while re-initialising a bus.
    void setSCL(int scl)
    {
        (void)scl;
    }

    void setSDA(int sda)
    {
        (void)sda;
    }

    void beginTransmission(uint8_t address) override
    {
        _txAddress = address;
        _txBufferLength = 0;
        _txOverflow = false;
        _transmitting = true;
    }

    // Error codes follow the core's own TwoWire::endTransmission: 0 success, 1 data too long,
    // 2 NACK on address, 3 NACK on data, 4 other error such as a bus held or a line stuck low.
    uint8_t endTransmission(bool stopBit) override
    {
        uint8_t result = 0;

        if (!i2c3_bb_start()) {
            result = 4; // no START possible: SDA is being held, no transaction was begun
        } else if (!i2c3_bb_write_byte(_txAddress << 1)) {
            i2c3_bb_stop();
            result = 2;
        } else {
            for (size_t i = 0; i < _txBufferLength; ++i) {
                if (!i2c3_bb_write_byte(_txBuffer[i])) {
                    i2c3_bb_stop();
                    result = 3;
                    break;
                }
            }
            if (result == 0) {
                if (!i2c3_bb_bus_released()) {
                    i2c3_bb_stop();
                    result = 4;
                } else if (stopBit) {
                    i2c3_bb_stop();
                }
            }
        }

        // A truncated payload outranks a clean transfer but never a NACK, which is the more
        // actionable diagnosis of the two.
        if (result == 0 && _txOverflow) {
            result = 1;
        }

        _txBufferLength = 0;
        _txOverflow = false;
        _transmitting = false;
        return result;
    }

    uint8_t endTransmission(void) override
    {
        return endTransmission(true);
    }

    // Returns the number of bytes actually clocked, so a bus taken over mid-transfer reads as a
    // short count rather than as a full buffer of 0xFF. The rx buffer is cleared on entry the
    // same way endTransmission clears the tx one, leaving no stale byte behind a failed read.
    size_t requestFrom(uint8_t address, size_t len, bool stopBit) override
    {
        _rxBufferIndex = 0;
        _rxBufferLength = 0;

        if (len > sizeof(_rxBuffer)) {
            len = sizeof(_rxBuffer);
        }
        if (!i2c3_bb_start()) {
            return 0;
        }
        if (!i2c3_bb_write_byte((address << 1) | 0x01)) {
            i2c3_bb_stop();
            return 0;
        }

        size_t received = 0;
        for (size_t i = 0; i < len; ++i) {
            _rxBuffer[i] = i2c3_bb_read_byte(i < len - 1);
            if (!i2c3_bb_bus_released()) {
                break; // SCL held: this byte and everything after it are meaningless
            }
            ++received;
        }

        if (stopBit) {
            i2c3_bb_stop();
        }
        _rxBufferLength = received;
        return received;
    }

    size_t requestFrom(uint8_t address, size_t len) override
    {
        return requestFrom(address, len, true);
    }

    size_t write(uint8_t data) override
    {
        if (!_transmitting) {
            return 0; // matches the core: bytes outside a transmission are not buffered
        }
        if (_txBufferLength < sizeof(_txBuffer)) {
            _txBuffer[_txBufferLength++] = data;
            return 1;
        }
        _txOverflow = true;
        return 0;
    }

    size_t write(const uint8_t *data, size_t quantity) override
    {
        size_t written = 0;
        for (size_t i = 0; i < quantity; ++i) {
            if (write(data[i])) {
                ++written;
            } else {
                break;
            }
        }
        return written;
    }

    int available(void) override
    {
        return _rxBufferLength - _rxBufferIndex;
    }

    int read(void) override
    {
        if (_rxBufferIndex < _rxBufferLength) {
            return _rxBuffer[_rxBufferIndex++];
        }
        return -1;
    }

    int peek(void) override
    {
        if (_rxBufferIndex < _rxBufferLength) {
            return _rxBuffer[_rxBufferIndex];
        }
        return -1;
    }

    // Nothing to flush: endTransmission clocks the buffered bytes itself, so no write is ever
    // pending once it returns. Harmless in this application, where the only consumer is a
    // register-based ADC read one transaction at a time.
    void flush(void) override
    {
    }

private:
    uint8_t _txAddress;
    uint8_t _txBuffer[32];
    size_t _txBufferLength;
    bool _txOverflow; // a write was refused because the payload exceeded the buffer
    bool _transmitting; // set by beginTransmission, cleared by endTransmission
    uint8_t _rxBuffer[32];
    size_t _rxBufferIndex;
    size_t _rxBufferLength;
};

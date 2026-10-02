#pragma once

#include "i2c3_bitbang.h"

#include <Arduino.h>
#include <Wire.h>
#include <cstdint>

class BitBangWire : public TwoWire {
public:
    BitBangWire(int sda, int scl)
            : TwoWire(sda, scl)
            , _txAddress(0)
            , _txBufferLength(0)
            , _rxBufferIndex(0)
            , _rxBufferLength(0)
    {
    }

    void begin()
    {
        i2c3_bb_init();
    }

    void begin(uint8_t address)
    {
        (void)address;
        i2c3_bb_init();
    }

    void begin(int address)
    {
        begin((uint8_t)address);
    }

    void end()
    {
        i2c3_bb_init();
    }

    void setClock(uint32_t frequency)
    {
        (void)frequency;
    }

    void setSCL(int scl)
    {
        (void)scl;
    }

    void setSDA(int sda)
    {
        (void)sda;
    }

    void beginTransmission(uint8_t address)
    {
        _txAddress = address;
        _txBufferLength = 0;
    }

    void beginTransmission(int address)
    {
        beginTransmission((uint8_t)address);
    }

    uint8_t endTransmission(bool stopBit)
    {
        if (!i2c3_bb_start()) {
            return 2;
        }
        if (!i2c3_bb_write_byte(_txAddress << 1)) {
            i2c3_bb_stop();
            return 2;
        }
        for (size_t i = 0; i < _txBufferLength; ++i) {
            if (!i2c3_bb_write_byte(_txBuffer[i])) {
                i2c3_bb_stop();
                return 2;
            }
        }
        if (stopBit) {
            i2c3_bb_stop();
        }
        return 0;
    }

    uint8_t endTransmission(void)
    {
        return endTransmission(true);
    }

    size_t write(uint8_t data)
    {
        if (_txBufferLength < sizeof(_txBuffer)) {
            _txBuffer[_txBufferLength++] = data;
            return 1;
        }
        return 0;
    }

    size_t write(const uint8_t *data, size_t quantity)
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

    size_t requestFrom(uint8_t address, size_t len, bool stopBit)
    {
        (void)stopBit;
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
        for (size_t i = 0; i < len; ++i) {
            _rxBuffer[i] = i2c3_bb_read_byte(i < len - 1);
        }
        i2c3_bb_stop();
        _rxBufferIndex = 0;
        _rxBufferLength = len;
        return len;
    }

    size_t requestFrom(uint8_t address, size_t len)
    {
        return requestFrom(address, len, true);
    }

    size_t requestFrom(int address, int quantity, int sendStop)
    {
        return requestFrom((uint8_t)address, (size_t)quantity, (bool)sendStop);
    }

    size_t requestFrom(int address, int quantity)
    {
        return requestFrom((uint8_t)address, (size_t)quantity);
    }

    int available(void)
    {
        return _rxBufferLength - _rxBufferIndex;
    }

    int read(void)
    {
        if (_rxBufferIndex < _rxBufferLength) {
            return _rxBuffer[_rxBufferIndex++];
        }
        return -1;
    }

    int peek(void)
    {
        if (_rxBufferIndex < _rxBufferLength) {
            return _rxBuffer[_rxBufferIndex];
        }
        return -1;
    }

    void flush(void)
    {
    }

private:
    uint8_t _txAddress;
    uint8_t _txBuffer[32];
    size_t _txBufferLength;
    uint8_t _rxBuffer[32];
    size_t _rxBufferIndex;
    size_t _rxBufferLength;
};

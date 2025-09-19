#pragma once

#include <cstddef>
#include <stdint.h>

// Stub implementation of Wire (I2C) for PC emulation
class TwoWire {
public:
    void begin()
    {
        // Stub: do nothing
    }

    void begin(uint8_t address)
    {
        // Stub: do nothing
    }

    void beginTransmission(uint8_t address)
    {
        // Stub: do nothing
    }

    uint8_t endTransmission()
    {
        // Stub: always succeed
        return 0;
    }

    uint8_t requestFrom(uint8_t address, uint8_t quantity)
    {
        // Stub: return requested quantity
        return quantity;
    }

    size_t write(uint8_t data)
    {
        // Stub: always succeed
        return 1;
    }

    size_t write(const uint8_t *data, size_t quantity)
    {
        // Stub: always succeed
        return quantity;
    }

    int available()
    {
        // Stub: no data available
        return 0;
    }

    int read()
    {
        // Stub: return dummy data
        return 0;
    }

    int peek()
    {
        // Stub: return dummy data
        return 0;
    }

    void flush()
    {
        // Stub: do nothing
    }

    void setClock(uint32_t clockFrequency)
    {
        // Stub: do nothing
    }
};

// Global Wire object
extern TwoWire Wire;

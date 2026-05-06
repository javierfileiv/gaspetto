#pragma once

#include <cstddef>
#include <stdint.h>

// Stub implementation of Wire (I2C) for PC emulation
class TwoWire {
public:
    TwoWire() = default;

    TwoWire(int sdaPin, int sclPin)
            : sclPin_(sclPin)
            , sdaPin_(sdaPin)
    {
    }

    void begin()
    {
        // Stub: do nothing
    }

    void begin(uint8_t address)
    {
        // Stub: do nothing
    }

    void end()
    {
        // Stub: do nothing
    }

    void beginTransmission(uint8_t address)
    {
        lastTransmissionAddress_ = address;
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
        lastWrittenByte_ = data;
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

    void setSCL(int pin)
    {
        // Stub: remember configured pin for diagnostics.
        sclPin_ = pin;
    }

    void setSDA(int pin)
    {
        // Stub: remember configured pin for diagnostics.
        sdaPin_ = pin;
    }

    uint8_t lastWrittenByte() const
    {
        return lastWrittenByte_;
    }

private:
    uint8_t lastTransmissionAddress_ = 0;
    uint8_t lastWrittenByte_ = 1;
    int sclPin_ = -1;
    int sdaPin_ = -1;
};

// Global Wire object
extern TwoWire Wire;
extern TwoWire Wire3;

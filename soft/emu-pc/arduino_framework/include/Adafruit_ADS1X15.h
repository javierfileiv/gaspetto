#pragma once

#include "Wire.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

constexpr int GAIN_ONE = 1;

class Adafruit_ADS1115 {
public:
    bool begin(uint8_t address = 0x48, TwoWire *wire = nullptr)
    {
        const char *forceFail = std::getenv("ADS_BEGIN_FAIL");

        if ((forceFail != nullptr) && (std::strcmp(forceFail, "1") == 0)) {
            return false;
        }
        currentAddress_ = address;
        currentWire_ = wire != nullptr ? wire : &Wire;
        return slotBaseFor(currentWire_, currentAddress_) < rawValues().size();
    }

    void setGain(int)
    {
    }

    int16_t readADC_SingleEnded(uint8_t channel)
    {
        const std::size_t slot = slotBaseFor(currentWire_, currentAddress_) + channel;
        if (slot >= rawValues().size()) {
            return 0;
        }
        return rawValues()[slot];
    }

    static void setRawValue(std::size_t slot, int16_t value)
    {
        if (slot < rawValues().size()) {
            rawValues()[slot] = value;
        }
    }

    static void setRawValues(const std::array<uint16_t, 20> &values)
    {
        for (std::size_t slot = 0; slot < values.size(); ++slot) {
            rawValues()[slot] = static_cast<int16_t>(values[slot]);
        }
    }

    static void resetPattern()
    {
        rawValues() = {
            400,  1000, 1600, 2200, 2800, 3400, 40,   400,  1000, 1600,
            2200, 2800, 3400, 40,   400,  1000, 1600, 2200, 2800, 3400,
        };
    }

private:
    static std::size_t slotBaseFor(const TwoWire *wire, uint8_t address)
    {
        if (wire != &Wire) {
            return address == 0x48 ? 16u : rawValues().size();
        }

        switch (address) {
        case 0x48:
            return 0u;
        case 0x49:
            return 4u;
        case 0x4A:
            return 8u;
        case 0x4B:
            return 12u;
        default:
            return rawValues().size();
        }
    }

    static std::array<int16_t, 20> &rawValues()
    {
        static std::array<int16_t, 20> values = {
            400,  1000, 1600, 2200, 2800, 3400, 40,   400,  1000, 1600,
            2200, 2800, 3400, 40,   400,  1000, 1600, 2200, 2800, 3400,
        };
        return values;
    }

    TwoWire *currentWire_ = &Wire;
    uint8_t currentAddress_ = 0x48;
};

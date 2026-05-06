#pragma once

#include "Wire.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern std::atomic<unsigned long> microsCounter;

constexpr int GAIN_ONE = 1;
constexpr uint8_t ADS1X15_REG_POINTER_CONFIG = 0x01;
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_0 = 0x4000;
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_1 = 0x5000;
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_2 = 0x6000;
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_3 = 0x7000;

class Adafruit_ADS1115 {
public:
    bool begin(uint8_t address = 0x48, TwoWire *wire = nullptr)
    {
        if (envFlag("ADS_BEGIN_FAIL")) {
            return false;
        }
        currentAddress_ = address;
        currentWire_ = wire != nullptr ? wire : &Wire;
        currentChannel_ = 0;
        lastConversionResult_ = 0;
        conversionStarted_ = false;
        return slotBaseFor(currentWire_, currentAddress_) < rawValues().size();
    }

    void setGain(int)
    {
    }

    int16_t readADC_SingleEnded(uint8_t channel)
    {
        const std::size_t slot = slotBaseFor(currentWire_, currentAddress_) + channel;
        if (envFlag("ADS_FORCE_NEGATIVE_RAW")) {
            return -1;
        }
        if (slot >= rawValues().size()) {
            return 0;
        }
        return rawValues()[slot];
    }

    void startADCReading(uint16_t mux, bool)
    {
        currentChannel_ = channelForMux(mux);
        lastConversionResult_ = readADC_SingleEnded(currentChannel_);
        conversionStarted_ = true;
    }

    bool conversionComplete()
    {
        if (envFlag("ADS_CONVERSION_STUCK")) {
            microsCounter.fetch_add(100000UL);
            return false;
        }
        return conversionStarted_;
    }

    int16_t getLastConversionResults()
    {
        return lastConversionResult_;
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
    static bool envFlag(const char *name)
    {
        const char *value = std::getenv(name);
        return (value != nullptr) && (std::strcmp(value, "1") == 0);
    }

    static std::size_t slotBaseFor(const TwoWire *wire, uint8_t address)
    {
        if (wire != &Wire) {
            return address == 0x4A ? 16u : rawValues().size();
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

    static uint8_t channelForMux(uint16_t mux)
    {
        switch (mux) {
        case ADS1X15_REG_CONFIG_MUX_SINGLE_0:
            return 0u;
        case ADS1X15_REG_CONFIG_MUX_SINGLE_1:
            return 1u;
        case ADS1X15_REG_CONFIG_MUX_SINGLE_2:
            return 2u;
        case ADS1X15_REG_CONFIG_MUX_SINGLE_3:
            return 3u;
        default:
            return 0u;
        }
    }

    TwoWire *currentWire_ = &Wire;
    uint8_t currentAddress_ = 0x48;
    uint8_t currentChannel_ = 0;
    int16_t lastConversionResult_ = 0;
    bool conversionStarted_ = false;
};

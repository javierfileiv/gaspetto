/* I2C bus layer: the ADS1115 probe list, raw and config reads with
 * per-bus recovery, calibration statistics and the health-cycle probe
 * comparison shared with the main program */
#include "hwtest.h"
#include "i2c3_bitbang.h"

#include <Adafruit_ADS1X15.h>

// ==========================================
// BASIC I2C PROBE (always enabled)
// Checks each expected ADS1115 address responds with ACK.
// ==========================================

struct I2CProbeEntry
{
    bool useBitBang; // true = use bit-banging, false = use TwoWire
    TwoWire *wire;   // only used if useBitBang is false
    uint8_t address;
    const char *label;
};

/* Board wiring (slot ranges fixed by the physical layout):
 * slots 1-4 ADS at 0x4B (ADDR->SCL), 5-8 at 0x48 (ADDR->GND),
 * 9-12 at 0x4A (ADDR->SDA), 13-16 at 0x49 (ADDR->VCC),
 * 17-20 on I2C3 at 0x4A (ADDR->SDA). Entries follow slot order */
static const I2CProbeEntry kI2CProbes[] = {
    {false, &i2c1, 0x4B, "I2C1 0x4B (ADDR->SCL)"},  {false, &i2c1, 0x48, "I2C1 0x48 (ADDR->GND)"},
    {false, &i2c1, 0x4A, "I2C1 0x4A (ADDR->SDA)"},  {false, &i2c1, 0x49, "I2C1 0x49 (ADDR->VCC)"},
    {true, nullptr, 0x4A, "I2C3 0x4A (ADDR->SDA)"},
};
constexpr uint8_t kProbeCount = sizeof(kI2CProbes) / sizeof(kI2CProbes[0]);

void probeI2CDevices()
{
    Serial.println("\n--- ADS1115 I2C PROBE (3V3 rail ON) ---");
    for (uint8_t i = 0; i < kProbeCount; ++i)
    {
        const I2CProbeEntry &e = kI2CProbes[i];
        bool ack;

        if (e.useBitBang)
        {
            ack = i2c3_bb_probe(e.address);
        }
        else
        {
            e.wire->beginTransmission(e.address);
            ack = (e.wire->endTransmission() == 0);
        }

        if (ack)
        {
            Serial.print("   [OK]  ");
        }
        else
        {
            Serial.print("   [MISS] ");
        }
        Serial.println(e.label);
    }
    Serial.println("--- PROBE DONE ---");
}

#ifdef TEST_ADS1115
// Slot ranges (wired on the board): 1-4=0x4B(SCL), 5-8=0x48(GND),
// 9-12=0x4A(SDA), 13-16=0x49(VCC), 17-20=I2C3 0x4A (bit-bang)
AdsTestEntry kAdsTests[] = {
    {false, &i2c1, 0x4B, "I2C1", "ADDR->SCL"},  {false, &i2c1, 0x48, "I2C1", "ADDR->GND"},
    {false, &i2c1, 0x4A, "I2C1", "ADDR->SDA"},  {false, &i2c1, 0x49, "I2C1", "ADDR->VCC"},
    {true, nullptr, 0x4A, "I2C3", "ADDR->SDA"},
};

static_assert(sizeof(kAdsTests) / sizeof(kAdsTests[0]) == kAdsCount,
              "probe table must list all five ADS modules");

CalibrationStats gCalibrationStats[kCalibrationSlotCount] = {};
/* Change report history for the health cycle: the five ADS slots plus
 * the nRF24 chip state. Filled at the end of each health run and
 * compared against the previous run */
static bool gLastProbeState[kAdsCount + 1] = {};
static bool gProbeStateValid               = false;

void resetCalibrationStats()
{
    for (uint8_t i = 0; i < kCalibrationSlotCount; ++i)
    {
        gCalibrationStats[i] = CalibrationStats{};
    }
    gCalibrationScanCounter = 0;
}

/* Quiet variant of the probe used by the verbose one: same device list,
 * no printing, plus the nRF24 chip state as the last entry */
void reportProbeChanges()
{
    bool current[kAdsCount + 1];

    for (uint8_t i = 0; i < kAdsCount; ++i)
    {
        const AdsTestEntry &entry = kAdsTests[i];
        if (entry.useBitBang)
        {
            current[i] = i2c3_bb_probe(entry.address);
        }
        else
        {
            Adafruit_ADS1115 ads;
            current[i] = ads.begin(entry.address, entry.wire);
        }
    }
    current[kAdsCount] = radio.isChipConnected();

    if (gProbeStateValid)
    {
        bool unchanged = true;
        for (uint8_t i = 0; i <= kAdsCount; ++i)
        {
            if (current[i] == gLastProbeState[i])
            {
                continue;
            }
            unchanged = false;
            Serial.print("   [CHANGE] ");
            if (i < kAdsCount)
            {
                Serial.print(kAdsTests[i].busName);
                Serial.print(" 0x");
                Serial.print(kAdsTests[i].address, HEX);
            }
            else
            {
                Serial.print("nRF24L01+");
            }
            Serial.print(": ");
            Serial.print(gLastProbeState[i] ? "ok" : "miss");
            Serial.print(" -> ");
            Serial.println(current[i] ? "ok" : "miss");
        }
        if (unchanged)
        {
            Serial.println("   state unchanged since previous run");
        }
    }
    else
    {
        Serial.println("   baseline recorded (no previous run to compare)");
    }

    for (uint8_t i = 0; i <= kAdsCount; ++i)
    {
        gLastProbeState[i] = current[i];
    }
    gProbeStateValid = true;
}

bool isI2c3Entry(const AdsTestEntry *entry)
{
    return entry->useBitBang;
}

void recoverI2c3Bus()
{
    Serial.println("   [WARN] I2C3 transaction failed, reinitializing I2C3 bit-bang...");
    i2c3_bb_init();
    delay(20);
}

/* Bring one ADS module back after a failed transaction: bit-banged
 * I2C3 re-inits the bus itself; hardware I2C1 needs a bus restart and
 * a fresh module begin before the next try can succeed */
void recoverAdsModule(const AdsTestEntry *entry)
{
    if (entry->useBitBang)
    {
        recoverI2c3Bus();
        return;
    }

    entry->wire->end();
    entry->wire->begin();
    entry->wire->setClock(kI2c1ClockHz);
    Adafruit_ADS1115 ads;
    ads.begin(entry->address, entry->wire);
    ads.setGain(GAIN_ONE);
    Serial.println("   [RECOVERED] I2C1 module re-inited");
}

bool readAdsConfigRegister(const AdsTestEntry *entry, uint16_t &config)
{
    config = 0;

    if (entry->useBitBang)
    {
        return i2c3_bb_read_register(entry->address, ADS1X15_REG_POINTER_CONFIG, config);
    }

    TwoWire *wire   = entry->wire;
    uint8_t address = entry->address;

    // Transaction 1: Write register pointer
    wire->beginTransmission(address);
    wire->write(ADS1X15_REG_POINTER_CONFIG);
    const uint8_t txErr = wire->endTransmission(); // Stop condition (no repeated start)
    if (txErr != 0)
    {
        return false;
    }

    // Transaction 2: Read data
    const uint8_t rxCount = wire->requestFrom(address, static_cast<uint8_t>(2));
    if (rxCount != 2 || wire->available() < 2)
    {
        return false;
    }

    // Two read() calls in one expression would be unsequenced: the byte order of the
    // resulting word would then be unspecified. Sequence them explicitly.
    const uint16_t high = static_cast<uint16_t>(wire->read());
    const uint16_t low  = static_cast<uint16_t>(wire->read());
    config              = static_cast<uint16_t>((high << 8) | low);
    return true;
}

static bool readAdsSingleEnded(const AdsTestEntry *entry, uint8_t channel, int16_t &raw)
{
    static constexpr uint16_t kMuxByChannel[4] = {
        ADS1X15_REG_CONFIG_MUX_SINGLE_0,
        ADS1X15_REG_CONFIG_MUX_SINGLE_1,
        ADS1X15_REG_CONFIG_MUX_SINGLE_2,
        ADS1X15_REG_CONFIG_MUX_SINGLE_3,
    };

    if (channel > 3)
    {
        return false;
    }

    uint16_t config = ADS1X15_REG_CONFIG_CQUE_NONE | ADS1X15_REG_CONFIG_MODE_SINGLE |
                      ADS1X15_REG_CONFIG_PGA_4_096V | kMuxByChannel[channel] | RATE_ADS1115_128SPS |
                      ADS1X15_REG_CONFIG_OS_SINGLE;

    if (entry->useBitBang)
    {
        // Use bit-banging for I2C3
        if (!i2c3_bb_write_register(entry->address, ADS1X15_REG_POINTER_CONFIG, config))
        {
            return false;
        }
        delay(15);

        uint16_t conversion;
        if (!i2c3_bb_read_register(entry->address, ADS1X15_REG_POINTER_CONVERT, conversion))
        {
            return false;
        }
        raw = (int16_t)conversion;
        return true;
    }

    TwoWire *wire   = entry->wire;
    uint8_t address = entry->address;

    wire->beginTransmission(address);
    wire->write(ADS1X15_REG_POINTER_CONFIG);
    wire->write((uint8_t)(config >> 8));
    wire->write((uint8_t)(config & 0xFF));
    uint8_t txErr = wire->endTransmission();
    if (txErr != 0)
    {
        return false;
    }

    delay(15);

    wire->beginTransmission(address);
    wire->write(ADS1X15_REG_POINTER_CONVERT);
    txErr = wire->endTransmission();
    if (txErr != 0)
    {
        return false;
    }

    const uint8_t rxCount = wire->requestFrom(address, static_cast<uint8_t>(2));
    if (rxCount != 2 || wire->available() < 2)
    {
        return false;
    }

    const uint16_t high = static_cast<uint16_t>(wire->read());
    const uint16_t low  = static_cast<uint16_t>(wire->read());
    raw                 = static_cast<int16_t>((high << 8) | low);
    return true;
}

bool readAdsSingleEndedMean(const AdsTestEntry *entry, uint8_t channel, int16_t &meanRaw)
{
    int32_t sum = 0;
    for (uint8_t sample = 0; sample < kAdsMeanSamples; ++sample)
    {
        int16_t raw = 0;
        if (!readAdsSingleEnded(entry, channel, raw))
        {
            return false;
        }
        sum += raw;
    }

    meanRaw = static_cast<int16_t>(sum / static_cast<int32_t>(kAdsMeanSamples));
    return true;
}

void updateCalibrationStats(uint8_t slotIndex, int16_t raw)
{
    CalibrationStats &stats = gCalibrationStats[slotIndex];
    if (!stats.initialized)
    {
        stats.initialized = true;
        stats.minRaw      = raw;
        stats.maxRaw      = raw;
        return;
    }

    if (raw < stats.minRaw)
    {
        stats.minRaw = raw;
    }
    if (raw > stats.maxRaw)
    {
        stats.maxRaw = raw;
    }
}

#endif /* TEST_ADS1115 */

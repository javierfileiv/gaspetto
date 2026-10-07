#include "BitBangWire.h"
#include "i2c3_bitbang.h"
#include "pin_definitions.h"

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <Wire.h>

static const uint8_t kAds1115Address      = 0x4A;
static const uint16_t kAds1115ResetConfig = 0x8583;
static const uint16_t kRailSettleDelayMs  = 250;

// Production uses 40ms (GASPETTO_ADC_CONVERSION_TIMEOUT_MS). Kept generous here because a
// poll is a full bit-bang transaction, about 1.4ms at 33kHz, and the conversion itself
// takes 7.8ms at the data rate the Adafruit library defaults to.
static const uint32_t kAdcConversionTimeoutMs = 150;

static int gPassed = 0;
static int gFailed = 0;

static void reportResult(const char *name, bool ok)
{
    if (ok)
    {
        Serial.print("   [OK]   ");
        gPassed++;
    }
    else
    {
        Serial.print("   [FAIL] ");
        gFailed++;
    }
    Serial.println(name);
}

// The instance under test: the bit-bang standing in for the box BUS_3 wire.
BitBangWire gI2c3(I2C3_SDA_PIN, I2C3_SCL_PIN);

// Reference path. These helpers drive the wrapper at its static type, so they reach the
// bit-bang no matter how dispatch behaves. They deliberately repeat, through the C++
// interface, what i2c3_bb_write_register() and i2c3_bb_read_register() already do in C:
// comparing an Adafruit object against this path then isolates exactly one variable, the
// dispatch through TwoWire*, instead of also re-testing the driver.
static bool bitBangWriteRegister(uint8_t address, uint8_t reg, uint16_t value)
{
    gI2c3.beginTransmission(address);
    gI2c3.write(reg);
    gI2c3.write((uint8_t)(value >> 8));
    gI2c3.write((uint8_t)(value & 0xFF));
    return gI2c3.endTransmission() == 0;
}

static bool bitBangReadRegister(uint8_t address, uint8_t reg, uint16_t &value)
{
    gI2c3.beginTransmission(address);
    gI2c3.write(reg);
    if (gI2c3.endTransmission() != 0)
    {
        return false;
    }
    if (gI2c3.requestFrom(address, (size_t)2) != 2)
    {
        return false;
    }
    if (gI2c3.available() < 2)
    {
        return false;
    }
    const int high = gI2c3.read();
    const int low  = gI2c3.read();
    value          = ((uint16_t)high << 8) | (uint16_t)low;
    return true;
}

// Adafruit's own readADC_SingleEnded() waits with `while (!conversionComplete()) ;`,
// which cannot report a bus failure: readRegister() drops the error code and builds its
// return value with the register number in the high byte, so a failed read of CONFIG
// yields 0x01xx, whose OS bit (15) is always 0. The library loop then never exits.
// Same sequence, bounded.
static bool readSingleEndedBounded(Adafruit_ADS1115 &ads, uint8_t channel, int16_t &raw)
{
    ads.startADCReading(MUX_BY_CHANNEL[channel], false);
    uint32_t startMs = millis();
    while (!ads.conversionComplete())
    {
        if (millis() - startMs > kAdcConversionTimeoutMs)
        {
            return false;
        }
        delay(1);
    }
    raw = ads.getLastConversionResults();
    return true;
}

void setup()
{
    delay(3000);
    Serial.begin(115200);
    Serial.println("\n\n--- BITBANGWIRE + ADS1115 TEST STARTED ---");

    // Turn ON 3V3 sensor rail
    pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);
    Serial.println("3V3 sensor rail ON");
    delay(kRailSettleDelayMs);

    // Initialize BitBangWire (calls i2c3_bb_init)
    gI2c3.begin();
    delay(50);

    // ---- Test 1: probe via BitBangWire ----
    Serial.println("\n1. BitBangWire probe");
    gI2c3.beginTransmission(kAds1115Address);
    uint8_t err = gI2c3.endTransmission();
    reportResult("probe 0x4A responds ACK", err == 0);

    // ---- Test 2: read config register via BitBangWire ----
    Serial.println("\n2. BitBangWire read config register");
    gI2c3.beginTransmission(kAds1115Address);
    gI2c3.write(0x01); // config register pointer
    err       = gI2c3.endTransmission();
    bool txOk = (err == 0);

    size_t rxCount  = gI2c3.requestFrom(kAds1115Address, (size_t)2);
    bool rxOk       = (rxCount == 2 && gI2c3.available() >= 2);
    uint16_t config = 0;
    if (rxOk)
    {
        const int high = gI2c3.read();
        const int low  = gI2c3.read();
        config         = ((uint16_t)high << 8) | (uint16_t)low;
    }

    Serial.print("      config = 0x");
    Serial.println(config, HEX);
    reportResult("write register pointer", txOk);
    reportResult("read 2 bytes", rxOk);
    reportResult("config matches reset value 0x8583", config == kAds1115ResetConfig);

    // ---- Test 3: write + read-back config register via BitBangWire ----
    Serial.println("\n3. BitBangWire write + read-back config register");
    uint16_t testValue = 0xC0E3;

    gI2c3.beginTransmission(kAds1115Address);
    gI2c3.write(0x01);
    gI2c3.write((uint8_t)(testValue >> 8));
    gI2c3.write((uint8_t)(testValue & 0xFF));
    err          = gI2c3.endTransmission();
    bool writeOk = (err == 0);

    gI2c3.beginTransmission(kAds1115Address);
    gI2c3.write(0x01);
    gI2c3.endTransmission();
    rxCount           = gI2c3.requestFrom(kAds1115Address, (size_t)2);
    uint16_t readBack = 0;
    if (rxCount == 2 && gI2c3.available() >= 2)
    {
        const int high = gI2c3.read();
        const int low  = gI2c3.read();
        readBack       = ((uint16_t)high << 8) | (uint16_t)low;
    }

    Serial.print("      wrote 0x");
    Serial.print(testValue, HEX);
    Serial.print(", read back 0x");
    Serial.println(readBack, HEX);
    reportResult("write config register", writeOk);
    reportResult("read back matches", (readBack & 0x7FFF) == (testValue & 0x7FFF));

    // Restore original config
    gI2c3.beginTransmission(kAds1115Address);
    gI2c3.write(0x01);
    gI2c3.write((uint8_t)(kAds1115ResetConfig >> 8));
    gI2c3.write((uint8_t)(kAds1115ResetConfig & 0xFF));
    gI2c3.endTransmission();

    // ---- Test 4: Adafruit_ADS1115 integration with BitBangWire ----
    Serial.println("\n4. Adafruit_ADS1115 integration via BitBangWire");
    Adafruit_ADS1115 ads;
    bool beginOk = ads.begin(kAds1115Address, &gI2c3);
    reportResult("ads.begin(0x4A, &gI2c3)", beginOk);

    if (beginOk)
    {
        ads.setGain(GAIN_ONE);

        // ---- Test 5: read 4 ADC channels ----
        Serial.println("\n5. ADC channel reads via Adafruit_ADS1115");
        bool allChannelsOk = true;
        for (uint8_t ch = 0; ch < 4; ++ch)
        {
            int16_t raw = 0;
            bool convOk = readSingleEndedBounded(ads, ch, raw);

            // Cross-check against the reference path: read the conversion
            // register again through gI2c3 and require the same value. A list
            // of rejected sentinels only guesses at the shape of the garbage;
            // this proves both paths see the same register.
            uint16_t directValue = 0;
            bool directOk =
                bitBangReadRegister(kAds1115Address, ADS1X15_REG_POINTER_CONVERT, directValue);
            int16_t direct = (int16_t)directValue;

            Serial.print("      ch");
            Serial.print(ch);
            Serial.print(": ");
            if (convOk)
            {
                Serial.print(raw);
            }
            else
            {
                Serial.print("conversion timeout");
            }
            bool chOk = convOk && directOk && (raw == direct);
            if (!chOk)
            {
                allChannelsOk = false;
                if (!convOk)
                {
                    Serial.print(
                        "  <-- CONVERSION NEVER COMPLETED, Adafruit write never reached the chip");
                }
                else if (!directOk)
                {
                    Serial.print("  <-- MISMATCH, bit-bang reference read failed");
                }
                else
                {
                    Serial.print("  <-- MISMATCH, bit-bang read = ");
                    Serial.print(direct);
                }
            }
            Serial.println();
            delay(10);
        }
        reportResult("all 4 channels match bit-bang reads", allChannelsOk);

        // ---- Test 6: does the Adafruit object reach the bit-bang? ----
        Serial.println("\n6. Adafruit register write reaches bit-bang");
        // Seed a known CONFIG (MUX = 0) through the reference path, then ask
        // the Adafruit object to rewrite it with MUX = AIN3. On this core
        // endTransmission() is not virtual, so a TwoWire* transmits an empty
        // payload: the register pointer never moves and the MUX field stays 0.
        bool seedOk =
            bitBangWriteRegister(kAds1115Address, ADS1X15_REG_POINTER_CONFIG, kAds1115ResetConfig);
        ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_3, false);
        uint16_t configAfter = 0;
        bool readOk = bitBangReadRegister(kAds1115Address, ADS1X15_REG_POINTER_CONFIG, configAfter);
        bool muxOk = (configAfter & ADS1X15_REG_CONFIG_MUX_MASK) == ADS1X15_REG_CONFIG_MUX_SINGLE_3;

        Serial.print("      CONFIG = 0x");
        Serial.println(configAfter, HEX);
        reportResult("seed CONFIG register", seedOk);
        reportResult("Adafruit write reaches bit-bang bus", readOk && muxOk);
    }

    // ---- Summary ----
    Serial.println("\n--- SUMMARY ---");
    Serial.print("PASSED: ");
    Serial.println(gPassed);
    Serial.print("FAILED: ");
    Serial.println(gFailed);
    Serial.print("TOTAL:  ");
    Serial.println(gPassed + gFailed);
    Serial.println(gFailed == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED");
    Serial.println("--- DONE ---");
}

void loop()
{
    delay(1000);
}

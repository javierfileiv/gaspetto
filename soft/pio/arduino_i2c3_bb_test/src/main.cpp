#include "BitBangWire.h"
#include "i2c3_bitbang.h"
#include "pin_definitions.h"

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <Wire.h>

static const uint8_t kAds1115Address      = 0x4A;
static const uint16_t kAds1115ResetConfig = 0x8583;
static const uint16_t kRailSettleDelayMs  = 250;

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

// BitBangWire gI2c3 must match GBox.cpp usage: BitBangWire on ARDUINO
BitBangWire gI2c3(I2C3_SDA_PIN, I2C3_SCL_PIN);

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
        config = ((uint16_t)gI2c3.read() << 8) | gI2c3.read();
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
        readBack = ((uint16_t)gI2c3.read() << 8) | gI2c3.read();
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
            int16_t raw = ads.readADC_SingleEnded(ch);
            Serial.print("      ch");
            Serial.print(ch);
            Serial.print(": ");
            Serial.print(raw);
            // Valid reading: not stuck at 0x7FFF or 0x8000 (bus failure artifacts)
            bool chOk = (raw != 32767 && raw != -32768);
            if (!chOk)
            {
                allChannelsOk = false;
                Serial.print("  <-- INVALID");
            }
            Serial.println();
            delay(10);
        }
        reportResult("all 4 channels readable", allChannelsOk);
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

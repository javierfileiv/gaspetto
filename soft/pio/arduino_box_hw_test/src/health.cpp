/* Health side of the bench: the ADS1115 bus test and the periodic cycle the
 * 's' key reruns */
#include <Arduino.h>

#ifdef TEST_ADS1115
#include <Adafruit_ADS1X15.h>
#endif

#include "hwtest.h"
#include "i2c3_bitbang.h"
#include "pin_definitions.h"

#ifdef TEST_ADS1115
/* ========================================== */
/* ADS1115 TEST */
/* ========================================== */

void runI2CTests()
{
    Serial.println("\n--- ADS1115 I2C TEST (3V3 rail ON) ---");

    for (uint8_t i = 0; i < kAdsCount; ++i)
    {
        const AdsTestEntry &entry = kAdsTests[i];

        /* Check if device is accessible */
        bool deviceFound;
        if (entry.useBitBang)
        {
            deviceFound = i2c3_bb_probe(entry.address);
        }
        else
        {
            Adafruit_ADS1115 ads;
            deviceFound = ads.begin(entry.address, entry.wire);
        }

        if (!deviceFound)
        {
            Serial.print("   [MISS] ");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.print(" (");
            Serial.print(entry.addrNote);
            Serial.println(") — not found");
            continue;
        }

        uint16_t config = 0;
        bool configOk   = readAdsConfigRegister(&entry, config);
        if (!configOk && isI2c3Entry(&entry))
        {
            recoverI2c3Bus();
            configOk = readAdsConfigRegister(&entry, config);
        }

        if (!configOk)
        {
            Serial.print("   [ERR] ");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.println(" config read failed (transaction error / short read)");
            continue;
        }

        Serial.print("          Config register: 0x");
        Serial.print(config, HEX);
        Serial.print(" (");
        Serial.print(config, BIN);
        Serial.print(") ");

        /* Compare with ADS1115 reset value (0x8583) */
        const uint16_t ADS1115_RESET_CONFIG = 0x8583;
        if (config == ADS1115_RESET_CONFIG)
        {
            Serial.println("[DEFAULT - OK]");
        }
        else
        {
            Serial.println("[CONFIGURED/NON-DEFAULT]");
        }

        Serial.print("   [OK]  ");
        Serial.print(entry.busName);
        Serial.print(" 0x");
        Serial.print(entry.address, HEX);
        Serial.print(" (");
        Serial.print(entry.addrNote);
        Serial.println("): communication check OK (config register read)");

#ifdef TEST_ADS1115_CONVERSION_READS
        Serial.print("          conversion path test enabled (timeout-guarded, mean samples=");
        Serial.print(kAdsMeanSamples);
        Serial.println(")");
        for (uint8_t ch = 0; ch < 4; ++ch)
        {
            int16_t rawMean = 0;
            bool readOk     = readAdsSingleEndedMean(&entry, ch, rawMean);
            if (!readOk && isI2c3Entry(&entry))
            {
                recoverI2c3Bus();
                readOk = readAdsSingleEndedMean(&entry, ch, rawMean);
            }

            Serial.print("          ch");
            Serial.print(ch);
            if (readOk)
            {
                Serial.print(": conversion OK (mean raw=");
                Serial.print(rawMean);
                Serial.println(")");
            }
            else
            {
                Serial.println(": conversion TIMEOUT/FAIL");
            }
        }
#endif
    }

    Serial.println("--- ADS1115 TEST DONE ---");
}
#endif /* TEST_ADS1115 */

#define DELAY 3000

/* ========================================== */
/* HEALTH CYCLE (runs on demand) */
/* ========================================== */
void runHealthCycle()
{
    /* Quick double blink visual check on builtin LED. */
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }

    Serial.println("[health] cycle start ('c' = calibration, '?' = status)");

    Serial.println("-> TURNING ON 5V_SWITCHED (LED Cache)...");
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW); /* Turn ON Q1 */

    /* The sensor rail is taken only for the probes: the hall sensors warm up
     * while it is up, and this cycle used to hold it through the blink and
     * the three second wait below for no reason - about half its rail-on time.
     * The acquire settles the rail itself, and every read inside sees the
     * flag already set, so it leaves the rail alone */
    const bool railHeld = sensorRailAcquire();

    probeI2CDevices();

#ifdef TEST_ADS1115
    /* Test ADS1115 modules while 3V3 is active */
    runI2CTests();
#endif

    /* The APA106 animations left this cycle: they cost several seconds each,
     * the LED menu ('l', key 0) runs them on demand instead */
    reportProbeChanges();

    sensorRailRelease(railHeld);

    /* Quick double blink visual check on builtin LED. */
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }
    delayWithUsb(DELAY);

    if (gMode != TestMode::Health)
    {
        /* Mode changed while waiting: leave the rails exactly as the new
         * mode set them. The sensor rail needs no care here - it went down
         * right after the probes - so only the LED rail is at stake */
        return;
    }

    Serial.println("-> TURNING OFF 5V_SWITCHED...");
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH); /* Turn OFF Q1 */

    Serial.println("Cycle complete. 's' reruns the test.");
    delayWithUsb(DELAY);
}

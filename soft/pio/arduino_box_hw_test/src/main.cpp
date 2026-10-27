#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <config_radio.h>

#ifdef TEST_ADS1115
#include <Adafruit_ADS1X15.h>
#endif
#include "CommandPacket.h"
#include "dfu_bootloader.h"
#include "hwtest.h"
#include "i2c3_bitbang.h"
#include "piece_thresholds.h"
#include "pin_definitions.h"

// Initialize radio object
RF24 radio(NRF24_CE, NRF24_CSN);

// I2C1 uses the global Wire instance
TwoWire &i2c1 = Wire;

constexpr uint16_t kRailSettleDelayMs       = 250;
constexpr uint16_t kCalibrationScanPeriodMs = 1000;

#ifdef TEST_ADS1115

TestMode gMode           = TestMode::Health;
bool gCalibrationRailsOn = false;
/* One test run is pending, set by 's' from any mode. False at boot: the
 * board waits for a key instead of running the health cycle on its own */
bool gRunTestRequested = false;

void enterMode(TestMode mode)
{
    if (gMode == mode)
    {
        gRunTestRequested = true; /* already active: re-run this mode test */
        Serial.println("\n-> test already active, running it ('?' status)");
        return;
    }

    gMode = mode;
    if (gMode == TestMode::Calibration)
    {
        resetCalibrationStats();
        /* Force the rail state right away so the first scan sees powered
         * sensors regardless of where the health cycle was interrupted */
        digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
        digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);
        gCalibrationRailsOn = true;
        delay(kRailSettleDelayMs);
        Serial.println("\n-> mode: calibration ('s' scan, 't' gaps, '?' status)");
    }
    else
    {
        digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
        digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
        gCalibrationRailsOn = false;
        Serial.println("\n-> mode: health ('?' for status)");
    }
}

// ==========================================
// ADS1115 TEST
// ==========================================

void runI2CTests()
{
    Serial.println("\n--- ADS1115 I2C TEST (3V3 rail ON) ---");

    for (uint8_t i = 0; i < kAdsCount; ++i)
    {
        const AdsTestEntry &entry = kAdsTests[i];

        // Check if device is accessible
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

        // Compare with ADS1115 reset value (0x8583)
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
#endif // TEST_ADS1115

#define DELAY 3000

// ==========================================
// SETUP
// ==========================================
void setup()
{
    // Initialize USB CDC Serial Monitor
    Serial.begin(115200);
    delay(3000); // Wait for Serial Monitor to open
    Serial.println("\n\n--- HARDWARE TEST STARTED ---");

#ifdef TEST_LED_ANIMATIONS
    // --- APA106 LED Configuration ---
    pinMode(PIN_LED_DATA, OUTPUT);
    digitalWrite(PIN_LED_DATA, LOW);
    leds.begin();
    leds.clear();
    leds.show();
#endif

    /* The heartbeat writes PC13 from the first idle pass: without output
     * mode it toggles a floating input and the LED stays dark */
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); /* active-low: dark until the first blink */

    // --- MOSFETs Configuration (Active-Low Logic) ---
    // IMPORTANT: PB14 must be OPEN DRAIN to safely handle 5V!
    pinMode(PIN_MOSFET_5V_LEDS, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT_OPEN_DRAIN);

    // Turn OFF all rails by default (HIGH = OFF for P-Channel MOSFETs)
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    Serial.println("1. MOSFETs initialized and TURNED OFF.");

    // --- nRF24L01+ SPI Test ---
    Serial.println("\n2. Testing nRF24L01+ SPI communication...");

    if (radio.begin())
    {
        Serial.println("   [SUCCESS] nRF24L01+ detected! SPI is working.");
        radio.setPALevel(RF24_PA_LOW); // Set low power for testing
        radio.printDetails();          // Print chip info to Serial
    }
    else
    {
        Serial.println("   [ERROR] nRF24L01+ not found!");
        Serial.println("   -> Check soldering on PA3, PA4, PA5, PA6, PA7.");
        Serial.println("   -> Check if 3.3V is reaching the nRF board.");
    }

    // --- I2C init ---
    Serial.println("\n3. Initializing I2C buses...");
    i2c1.begin();
    i2c1.setClock(kI2c1ClockHz);
    Serial.println("   I2C1 (SCL=PB6, SDA=PB7) @ 100kHz initialized.");

    // Turn ON 3V3_SWITCHED FIRST so ADS1115 modules are powered
    Serial.println("\n4. Turning ON 3V3_SWITCHED to power ADS1115 modules...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW); // Turn ON Q2
    delay(kRailSettleDelayMs);                 // Wait for modules to initialize
    Serial.println("   [OK] 3V3_SWITCHED activated - ADS1115 modules should be powered");

    // Initialize I2C3 bit-banging
    Serial.println("\n5. Initializing I2C3 bit-banging (PB4=SDA, PA8=SCL)...");
    i2c3_bb_init();
    Serial.println("   [OK] I2C3 bit-banging initialized.");

    Serial.println("   NOTE: ADS1115 will be tested each cycle when 3V3 rail is ON.");

    Serial.println("   hw-test: runtime modes build");
    printUsbHelp();
    Serial.print("   mode: ");
    Serial.println(gMode == TestMode::Calibration ? "calibration" : "health");
    Serial.println("   waiting for a key ('?' for help)");
}

// ==========================================
// HEALTH CYCLE (runs on demand)
// ==========================================
void runHealthCycle()
{
    // Quick double blink visual check on builtin LED.
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }

    Serial.println("[health] cycle start ('c' = calibration, '?' = status)");

    Serial.println("-> TURNING ON 5V_SWITCHED (LED Cache)...");
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);     // Turn ON Q1
    Serial.println("-> TURNING ON 3V3_SWITCHED (Sensors)...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW); // Turn ON Q2

    delay(kRailSettleDelayMs);                 // Let rails settle before I2C

    probeI2CDevices();

#ifdef TEST_ADS1115
    // Test ADS1115 modules while 3V3 is active
    runI2CTests();
#endif

#ifdef TEST_LED_ANIMATIONS
    // APA106 animation must be sent while 5V LED rail is still powered.
    runPowerOnCycleAnimation();
    runPowerOffCycleAnimation();
#endif

    reportProbeChanges();

    // Quick double blink visual check on builtin LED.
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
        /* Mode changed while waiting: leave rails exactly as the new mode
         * set them instead of tearing the 3V3 rail down under it */
        return;
    }

    Serial.println("-> TURNING OFF 5V_SWITCHED...");
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);     // Turn OFF Q1
    Serial.println("-> TURNING OFF 3V3_SWITCHED...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH); // Turn OFF Q2

    Serial.println("Cycle complete. 's' reruns the test.");
    delayWithUsb(DELAY);
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop()
{
    handleUsbCommands();

    if (gMode == TestMode::Calibration)
    {
        if (!gCalibrationRailsOn)
        {
            Serial.println("-> TURNING ON 3V3_SWITCHED (Calibration sensors)...");
            digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);
            delay(kRailSettleDelayMs);
            gCalibrationRailsOn = true;
        }

        /* Game mode: auto-capture polls the current slot and records as
         * soon as the reading settles; otherwise just USB polling between
         * steps. EMPTY is excluded from auto (it needs one deliberate
         * SPACE so a board that still holds pieces is not recorded) */
        if (gGameActive)
        {
            if (gGameAuto && kGamePieces[gGamePieceIndex] != BoxPieceId::EMPTY)
            {
                pollGameAutoCapture();
            }
            delayWithUsb(50);
            return;
        }

        // Quick double blink visual check on builtin LED.
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(PIN_LED, LOW);
            delay(100);
            digitalWrite(PIN_LED, HIGH);
            delay(100);
        }

#ifdef TEST_ADS1115
        if (gRunTestRequested)
        {
            gRunTestRequested = false;
            runPieceCalibrationScan();
        }
#endif

        if (gMode != TestMode::Calibration)
        {
            return; /* mode changed while waiting: skip the rest delay */
        }

        delayWithUsb(kCalibrationScanPeriodMs);
        return;
    }

#ifdef TEST_ADS1115
    if (gRunTestRequested)
    {
        gRunTestRequested = false;
        runHealthCycle();
    }
#endif

    // Quiet idle: heartbeat only, no probes, no tests between demands.
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }
    delayWithUsb(kCalibrationScanPeriodMs);
}

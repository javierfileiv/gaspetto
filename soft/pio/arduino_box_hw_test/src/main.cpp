#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

#ifdef TEST_ADS1115
#include <Adafruit_ADS1X15.h>
#endif
#include "pin_definitions.h"

// Initialize radio object
RF24 radio(NRF24_CE, NRF24_CSN);

// I2C3 bus instance (I2C1 uses the global i2c1 alias)
TwoWire i2c3(I2C3_SDA_PIN, I2C3_SCL_PIN);
TwoWire &i2c1 = Wire;

constexpr uint32_t kI2c1ClockHz = 100000;
#ifndef TEST_I2C3_CLOCK_HZ
#define TEST_I2C3_CLOCK_HZ 50000
#endif
constexpr uint32_t kI2c3ClockHz            = TEST_I2C3_CLOCK_HZ;
constexpr uint16_t kRailSettleDelayMs      = 250;
constexpr uint16_t kAdsConversionTimeoutMs = 40;

#ifndef TEST_ADS1115_MEAN_SAMPLES
#define TEST_ADS1115_MEAN_SAMPLES 10
#endif

static_assert(TEST_ADS1115_MEAN_SAMPLES > 0, "TEST_ADS1115_MEAN_SAMPLES must be >= 1");

constexpr uint8_t kAdsMeanSamples = TEST_ADS1115_MEAN_SAMPLES;

// ==========================================
// BASIC I2C PROBE (always enabled)
// Checks each expected ADS1115 address responds with ACK.
// ==========================================

struct I2CProbeEntry
{
    TwoWire *wire;
    uint8_t address;
    const char *label;
};

static const I2CProbeEntry kI2CProbes[] = {
    {&i2c1, 0x48, "I2C1 0x48 (ADDR->GND)"}, {&i2c1, 0x49, "I2C1 0x49 (ADDR->VCC)"},
    {&i2c1, 0x4A, "I2C1 0x4A (ADDR->SDA)"}, {&i2c1, 0x4B, "I2C1 0x4B (ADDR->SCL)"},
    {&i2c3, 0x4A, "I2C3 0x4A (ADDR->SDA)"},
};
constexpr uint8_t kProbeCount = sizeof(kI2CProbes) / sizeof(kI2CProbes[0]);

void probeI2CDevices()
{
    Serial.println("\n--- ADS1115 I2C PROBE (3V3 rail ON) ---");
    for (uint8_t i = 0; i < kProbeCount; ++i)
    {
        const I2CProbeEntry &e = kI2CProbes[i];
        e.wire->beginTransmission(e.address);
        const uint8_t err = e.wire->endTransmission();
        if (err == 0)
        {
            Serial.print("   [OK]  ");
        }
        else
        {
            Serial.print("   [MISS] err=");
            Serial.print(err);
            Serial.print(" ");
        }
        Serial.println(e.label);
    }
    Serial.println("--- PROBE DONE ---");
}

#ifdef TEST_ADS1115
struct AdsTestEntry
{
    TwoWire *wire;
    uint8_t address;
    const char *busName;
    const char *addrNote;
};

// I2C1: ADDR→GND=0x48, ADDR→VCC=0x49, ADDR→SDA=0x4A, ADDR→SCL=0x4B
// I2C3: 1 module, ADDR→SDA=0x4A
static AdsTestEntry kAdsTests[] = {
    {&i2c1, 0x48, "I2C1", "ADDR->GND"}, {&i2c1, 0x49, "I2C1", "ADDR->VCC"},
    {&i2c1, 0x4A, "I2C1", "ADDR->SDA"}, {&i2c1, 0x4B, "I2C1", "ADDR->SCL"},
    {&i2c3, 0x4A, "I2C3", "ADDR->SDA"},
};
constexpr uint8_t kAdsCount = sizeof(kAdsTests) / sizeof(kAdsTests[0]);

bool isI2c3Wire(const TwoWire *wire)
{
    return wire == &i2c3;
}

void recoverI2c3Bus()
{
    Serial.println("   [WARN] I2C3 transaction failed, reinitializing I2C3 bus...");
    i2c3.end();
    delay(5);
    i2c3.begin();
    i2c3.setClock(kI2c3ClockHz);
    delay(20);
}

bool readAdsConfigRegister(TwoWire *wire, uint8_t address, uint16_t &config)
{
    config = 0;

    wire->beginTransmission(address);
    wire->write(ADS1X15_REG_POINTER_CONFIG);
    const uint8_t txErr = wire->endTransmission(false);
    if (txErr != 0)
    {
        return false;
    }

    const uint8_t rxCount = wire->requestFrom(address, static_cast<uint8_t>(2));
    if (rxCount != 2 || wire->available() < 2)
    {
        return false;
    }

    config = (wire->read() << 8) | wire->read();
    return true;
}

bool readAdsSingleEndedWithTimeout(Adafruit_ADS1115 &ads, uint8_t channel, int16_t &raw)
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

    ads.startADCReading(kMuxByChannel[channel], false);

    const unsigned long startMs = millis();
    while (!ads.conversionComplete())
    {
        if (millis() - startMs > kAdsConversionTimeoutMs)
        {
            return false;
        }
        delay(1);
    }

    raw = ads.getLastConversionResults();
    return true;
}

bool readAdsSingleEndedMeanWithTimeout(Adafruit_ADS1115 &ads, uint8_t channel, int16_t &meanRaw)
{
    int32_t sum = 0;
    for (uint8_t sample = 0; sample < kAdsMeanSamples; ++sample)
    {
        int16_t raw = 0;
        if (!readAdsSingleEndedWithTimeout(ads, channel, raw))
        {
            return false;
        }
        sum += raw;
    }

    meanRaw = static_cast<int16_t>(sum / static_cast<int32_t>(kAdsMeanSamples));
    return true;
}

// ==========================================
// ADS1115 TEST
// ==========================================

void runI2CTests()
{
    Serial.println("\n--- ADS1115 I2C TEST (3V3 rail ON) ---");
    Serial.println("   NOTE: ADC channels are floating in this test; values are not validated.");

    for (uint8_t i = 0; i < kAdsCount; ++i)
    {
        const AdsTestEntry &entry = kAdsTests[i];
        Adafruit_ADS1115 ads;

        if (!ads.begin(entry.address, entry.wire))
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

        ads.setGain(GAIN_ONE); // ±4.096 V, matches GaspettoBox

        uint16_t config = 0;
        bool configOk   = readAdsConfigRegister(entry.wire, entry.address, config);
        if (!configOk && isI2c3Wire(entry.wire))
        {
            recoverI2c3Bus();
            configOk = readAdsConfigRegister(entry.wire, entry.address, config);
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
            bool readOk     = readAdsSingleEndedMeanWithTimeout(ads, ch, rawMean);
            if (!readOk && isI2c3Wire(entry.wire))
            {
                recoverI2c3Bus();
                readOk = readAdsSingleEndedMeanWithTimeout(ads, ch, rawMean);
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

// ==========================================
// SETUP
// ==========================================
void setup()
{
    // Initialize USB CDC Serial Monitor
    Serial.begin(115200);
    delay(3000); // Wait for Serial Monitor to open
    Serial.println("\n\n--- HARDWARE TEST STARTED ---");

    // --- LED Configuration ---
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); // Turn OFF LED initially

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

    // Initialize SPI bus pins for STM32
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.setSCLK(PA5);
    SPI.begin();

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

    Serial.print("I2C3 object: ");
    Serial.println((uint32_t)&i2c3, HEX);
    // I2C3 is already configured in constructor with SDA=PB4, SCL=PA8
    i2c3.begin();
    delay(50); // Allow I2C3 to settle
    i2c3.setClock(kI2c3ClockHz);
    Serial.println("   I2C3 (SCL=PA8, SDA=PB4) @ 50kHz initialized.");

    Serial.println("   NOTE: ADS1115 will be tested each cycle when 3V3 rail is ON.");

    Serial.println("\n--- POWER RAILS TEST CYCLE ---");
}

// ==========================================
// MAIN LOOP (Test Cycle)
// ==========================================
void loop()
{
    // Quick double blink visual check on builtin LED.
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }

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

    // Quick double blink visual check on builtin LED.
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }
    delay(DELAY);

    Serial.println("-> TURNING OFF 5V_SWITCHED...");
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);     // Turn OFF Q1
    Serial.println("-> TURNING OFF 3V3_SWITCHED...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH); // Turn OFF Q2

    Serial.println("Cycle complete. Restarting in 2 seconds.");
    delay(DELAY);
}

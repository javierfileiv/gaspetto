#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <config_radio.h>

#ifdef TEST_LED_ANIMATIONS
#include <Adafruit_NeoPixel.h>
#endif
#ifdef TEST_ADS1115
#include <Adafruit_ADS1X15.h>
#endif
#include "i2c3_bitbang.h"
#include "pin_definitions.h"

// Initialize radio object
RF24 radio(NRF24_CE, NRF24_CSN);

// I2C1 uses the global Wire instance
TwoWire &i2c1 = Wire;

constexpr uint32_t kI2c1ClockHz             = 100000;
constexpr uint16_t kRailSettleDelayMs       = 250;
constexpr uint16_t kCalibrationScanPeriodMs = 1000;

#ifndef TEST_ADS1115_MEAN_SAMPLES
#define TEST_ADS1115_MEAN_SAMPLES 10
#endif

#ifndef TEST_THRESHOLD_FORWARD_START
#define TEST_THRESHOLD_FORWARD_START 100
#endif

#ifndef TEST_THRESHOLD_BACKWARD_START
#define TEST_THRESHOLD_BACKWARD_START 700
#endif

#ifndef TEST_THRESHOLD_TURN_RIGHT_START
#define TEST_THRESHOLD_TURN_RIGHT_START 1300
#endif

#ifndef TEST_THRESHOLD_TURN_LEFT_START
#define TEST_THRESHOLD_TURN_LEFT_START 1900
#endif

#ifndef TEST_THRESHOLD_LOOP_START
#define TEST_THRESHOLD_LOOP_START 2500
#endif

#ifndef TEST_THRESHOLD_LOOP_END
#define TEST_THRESHOLD_LOOP_END 3099
#endif

static_assert(TEST_ADS1115_MEAN_SAMPLES > 0, "TEST_ADS1115_MEAN_SAMPLES must be >= 1");
static_assert(TEST_THRESHOLD_FORWARD_START > 0, "Forward threshold must be > 0");
static_assert(TEST_THRESHOLD_FORWARD_START < TEST_THRESHOLD_BACKWARD_START,
              "Thresholds must be strictly increasing");
static_assert(TEST_THRESHOLD_BACKWARD_START < TEST_THRESHOLD_TURN_RIGHT_START,
              "Thresholds must be strictly increasing");
static_assert(TEST_THRESHOLD_TURN_RIGHT_START < TEST_THRESHOLD_TURN_LEFT_START,
              "Thresholds must be strictly increasing");
static_assert(TEST_THRESHOLD_TURN_LEFT_START < TEST_THRESHOLD_LOOP_START,
              "Thresholds must be strictly increasing");
static_assert(TEST_THRESHOLD_LOOP_START <= TEST_THRESHOLD_LOOP_END,
              "Loop threshold start must be <= loop threshold end");
static_assert(TEST_THRESHOLD_LOOP_END <= 4095, "Loop threshold end must be <= 4095");

constexpr uint8_t kAdsMeanSamples = TEST_ADS1115_MEAN_SAMPLES;

struct CalibrationStats
{
    bool initialized;
    int16_t minRaw;
    int16_t maxRaw;
};

#ifdef TEST_LED_ANIMATIONS
constexpr uint8_t kLedState = 0;
constexpr uint8_t kLedRadio = 1;
constexpr uint8_t kLedBuild = 2;
Adafruit_NeoPixel leds(BOX_LED_COUNT, PIN_LED_DATA, NEO_GRB + NEO_KHZ800);
#endif

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

static const I2CProbeEntry kI2CProbes[] = {
    {false, &i2c1, 0x48, "I2C1 0x48 (ADDR->GND)"},  {false, &i2c1, 0x49, "I2C1 0x49 (ADDR->VCC)"},
    {false, &i2c1, 0x4A, "I2C1 0x4A (ADDR->SDA)"},  {false, &i2c1, 0x4B, "I2C1 0x4B (ADDR->SCL)"},
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
struct AdsTestEntry
{
    bool useBitBang; // true = use bit-banging, false = use TwoWire
    TwoWire *wire;   // only used if useBitBang is false
    uint8_t address;
    const char *busName;
    const char *addrNote;
};

// I2C1: ADDR→GND=0x48, ADDR→VCC=0x49, ADDR→SDA=0x4A, ADDR→SCL=0x4B
// I2C3: 1 module, ADDR→SDA=0x4A (uses bit-banging)
static AdsTestEntry kAdsTests[] = {
    {false, &i2c1, 0x48, "I2C1", "ADDR->GND"},  {false, &i2c1, 0x49, "I2C1", "ADDR->VCC"},
    {false, &i2c1, 0x4A, "I2C1", "ADDR->SDA"},  {false, &i2c1, 0x4B, "I2C1", "ADDR->SCL"},
    {true, nullptr, 0x4A, "I2C3", "ADDR->SDA"},
};
constexpr uint8_t kAdsCount             = sizeof(kAdsTests) / sizeof(kAdsTests[0]);
constexpr uint8_t kCalibrationSlotCount = kAdsCount * 4;

CalibrationStats gCalibrationStats[kCalibrationSlotCount] = {};
uint32_t gCalibrationScanCounter                          = 0;

/* Runtime test modes, switched over the USB CDC port. Health runs the
 * connectivity cycle each pass; Calibration keeps the 3V3 sensor rail
 * powered and watches piece ADC values continuously */
enum class TestMode : uint8_t
{
    Health,
    Calibration,
};

TestMode gMode           = TestMode::Health;
bool gCalibrationRailsOn = false;
/* One test run is pending. True at boot so the first health pass runs
 * without a keystroke, then set again by 's' from any mode */
bool gRunTestRequested = true;

/* Change report history for the health cycle: the five ADS slots plus
 * the nRF24 chip state. Filled at the end of each health run and
 * compared against the previous run */
bool gLastProbeState[kAdsCount + 1] = {};
bool gProbeStateValid               = false;

/* Sliced delay so USB CDC commands are polled while the test cycles wait.
 * Rail settle delays keep the plain delay(): the electrical settling must
 * not be interrupted */
void handleUsbCommands();

void delayWithUsb(uint32_t totalMs)
{
    constexpr uint32_t kSliceMs = 50;
    uint32_t remaining          = totalMs;
    while (remaining > 0)
    {
        const uint32_t slice = remaining > kSliceMs ? kSliceMs : remaining;
        delay(slice);
        remaining -= slice;
        handleUsbCommands();
    }
}

void resetCalibrationStats()
{
    for (uint8_t i = 0; i < kCalibrationSlotCount; ++i)
    {
        gCalibrationStats[i] = CalibrationStats{};
    }
    gCalibrationScanCounter = 0;
}

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
        Serial.println("\n-> mode: calibration ('s' scan, '?' status)");
    }
    else
    {
        digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
        digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
        gCalibrationRailsOn = false;
        Serial.println("\n-> mode: health ('?' for status)");
    }
}

void handleUsbCommands()
{
    while (Serial.available() > 0)
    {
        const int incoming = Serial.read();
        switch (incoming)
        {
        case 'h':
            enterMode(TestMode::Health);
            break;
        case 'c':
            enterMode(TestMode::Calibration);
            break;
        case 's':
            gRunTestRequested = true;
            break;
        default:
            break;
        }
    }
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

bool readAdsSingleEnded(const AdsTestEntry *entry, uint8_t channel, int16_t &raw)
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

const char *pieceLabelForRaw(int16_t raw)
{
    if (raw < 0)
    {
        return "INVALID";
    }
    if (raw < TEST_THRESHOLD_FORWARD_START)
    {
        return "EMPTY";
    }
    if (raw < TEST_THRESHOLD_BACKWARD_START)
    {
        return "FORWARD";
    }
    if (raw < TEST_THRESHOLD_TURN_RIGHT_START)
    {
        return "BACKWARD";
    }
    if (raw < TEST_THRESHOLD_TURN_LEFT_START)
    {
        return "TURN_RIGHT";
    }
    if (raw < TEST_THRESHOLD_LOOP_START)
    {
        return "TURN_LEFT";
    }
    if (raw <= TEST_THRESHOLD_LOOP_END)
    {
        return "LOOP_CALL";
    }
    return "INVALID";
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

void runPieceCalibrationScan()
{
    Serial.println("\n--- PIECE THRESHOLD CALIBRATION ---");
    Serial.print("scan=");
    Serial.println(++gCalibrationScanCounter);
    Serial.print("mean samples=");
    Serial.print(kAdsMeanSamples);
    Serial.print(" thresholds=[FWD:");
    Serial.print(TEST_THRESHOLD_FORWARD_START);
    Serial.print(" BACK:");
    Serial.print(TEST_THRESHOLD_BACKWARD_START);
    Serial.print(" RIGHT:");
    Serial.print(TEST_THRESHOLD_TURN_RIGHT_START);
    Serial.print(" LEFT:");
    Serial.print(TEST_THRESHOLD_TURN_LEFT_START);
    Serial.print(" LOOP:");
    Serial.print(TEST_THRESHOLD_LOOP_START);
    Serial.print(" LOOPEND:");
    Serial.print(TEST_THRESHOLD_LOOP_END);
    Serial.println("]");

    for (uint8_t deviceIndex = 0; deviceIndex < kAdsCount; ++deviceIndex)
    {
        const AdsTestEntry &entry = kAdsTests[deviceIndex];

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
            Serial.println(" calibration scan skipped");
            continue;
        }

        for (uint8_t ch = 0; ch < 4; ++ch)
        {
            int16_t rawMean = 0;
            bool readOk     = readAdsSingleEndedMean(&entry, ch, rawMean);

            if (!readOk && isI2c3Entry(&entry))
            {
                recoverI2c3Bus();
                readOk = readAdsSingleEndedMean(&entry, ch, rawMean);
            }

            const uint8_t slotIndex = static_cast<uint8_t>(deviceIndex * 4 + ch);
            Serial.print("   slot ");
            Serial.print(slotIndex + 1);
            Serial.print(" [");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.print(" ch");
            Serial.print(ch);
            Serial.print("] ");

            if (!readOk)
            {
                Serial.println("TIMEOUT/FAIL");
                continue;
            }

            updateCalibrationStats(slotIndex, rawMean);
            const CalibrationStats &stats = gCalibrationStats[slotIndex];

            Serial.print("mean=");
            Serial.print(rawMean);
            Serial.print(" min=");
            Serial.print(stats.minRaw);
            Serial.print(" max=");
            Serial.print(stats.maxRaw);
            Serial.print(" span=");
            Serial.print(stats.maxRaw - stats.minRaw);
            Serial.print(" => ");
            Serial.println(pieceLabelForRaw(rawMean));
        }
    }
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

#ifdef TEST_LED_ANIMATIONS
void blackoutLeds()
{
    leds.clear();
    leds.show();
}

void setOneLed(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
    leds.clear();
    leds.setPixelColor(led, leds.Color(r, g, b));
    leds.show();
}

void setTwoLeds(uint8_t ledA, uint8_t rA, uint8_t gA, uint8_t bA, uint8_t ledB, uint8_t rB,
                uint8_t gB, uint8_t bB)
{
    leds.clear();
    leds.setPixelColor(ledA, leds.Color(rA, gA, bA));
    leds.setPixelColor(ledB, leds.Color(rB, gB, bB));
    leds.show();
}

void runScanAnimationTest()
{
    // Mirrors GBox scan: cyan bounce across 3 LEDs, 2 round-trips.
    for (int trip = 0; trip < 2; ++trip)
    {
        for (int i = 0; i < 3; ++i)
        {
            setOneLed(static_cast<uint8_t>(i), 0, 80, 80);
            delay(120);
        }
        for (int i = 1; i >= 0; --i)
        {
            setOneLed(static_cast<uint8_t>(i), 0, 80, 80);
            delay(120);
        }
    }
    blackoutLeds();
}

void runSuccessAnimationTest()
{
    // Mirrors GBox success: green cascade then hold all green.
    leds.clear();
    for (int i = 0; i < 3; ++i)
    {
        leds.setPixelColor(static_cast<uint16_t>(i), leds.Color(0, 100, 0));
        leds.show();
        delay(150);
    }
    delay(1200);
    blackoutLeds();
}

void runBuildErrorAnimationTest()
{
    // Mirrors GBox build error: right LED red blink x3 then hold red.
    for (int i = 0; i < 3; ++i)
    {
        setOneLed(kLedBuild, 150, 0, 0);
        delay(90);
        blackoutLeds();
        delay(45);
    }
    setOneLed(kLedBuild, 150, 0, 0);
    delay(1200);
    blackoutLeds();
}

void runEmptyBoardAnimationTest()
{
    // Mirrors GBox empty board: right LED amber blink x2 then hold amber.
    for (int i = 0; i < 2; ++i)
    {
        setOneLed(kLedBuild, 100, 60, 0);
        delay(200);
        blackoutLeds();
        delay(100);
    }
    setOneLed(kLedBuild, 100, 60, 0);
    delay(1200);
    blackoutLeds();
}

void runRfErrorAnimationTest()
{
    // Mirrors GBox RF error: center red blink x3 then left green + center red.
    for (int i = 0; i < 3; ++i)
    {
        setOneLed(kLedRadio, 150, 0, 0);
        delay(90);
        blackoutLeds();
        delay(45);
    }
    setTwoLeds(kLedState, 0, 80, 0, kLedRadio, 150, 0, 0);
    delay(1200);
    blackoutLeds();
}

void runPowerOnCycleAnimation()
{
    runScanAnimationTest();
    runSuccessAnimationTest();
}

void runPowerOffCycleAnimation()
{
    runBuildErrorAnimationTest();
    runEmptyBoardAnimationTest();
    runRfErrorAnimationTest();
}
#endif

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
    Serial.println(
        "   USB commands: 'h' health mode, 'c' calibration mode, 's' run test, '?' status");
    Serial.print("   mode: ");
    Serial.println(gMode == TestMode::Calibration ? "calibration" : "health");

    Serial.println("\n--- POWER RAILS TEST CYCLE ---");
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

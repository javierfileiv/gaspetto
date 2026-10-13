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
#include "CommandPacket.h"
#include "i2c3_bitbang.h"
#include "piece_thresholds.h"
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

/* Piece thresholds come from the shared piece_thresholds.h with their
 * static asserts: same numbers as the production firmware */

static_assert(TEST_ADS1115_MEAN_SAMPLES > 0, "TEST_ADS1115_MEAN_SAMPLES must be >= 1");

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
struct AdsTestEntry
{
    bool useBitBang; // true = use bit-banging, false = use TwoWire
    TwoWire *wire;   // only used if useBitBang is false
    uint8_t address;
    const char *busName;
    const char *addrNote;
};

// Slot ranges (wired on the board): 1-4=0x4B(SCL), 5-8=0x48(GND),
// 9-12=0x4A(SDA), 13-16=0x49(VCC), 17-20=I2C3 0x4A (bit-bang)
static AdsTestEntry kAdsTests[] = {
    {false, &i2c1, 0x4B, "I2C1", "ADDR->SCL"},  {false, &i2c1, 0x48, "I2C1", "ADDR->GND"},
    {false, &i2c1, 0x4A, "I2C1", "ADDR->SDA"},  {false, &i2c1, 0x49, "I2C1", "ADDR->VCC"},
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

/* Continuous slot monitor ('m'): while active, usb keys only stop it */
bool gMonitorActive = false;
bool gMonitorExit   = false;

/* Game mode: guided calibration sequence. The operator is prompted for
 * each (piece, slot) combination and presses SPACE to record once the
 * reading stabilizes. The kGamePieces array lists which pieces are
 * iterated; bumping kGamePieceCount adds a new piece (e.g. LOOP_CALL
 * when the orange magnet is fabricated) without touching the rest.
 * kGameSlotCount is the number of slots covered (currently 4 because
 * only slots 1-4 have sensors soldered, extensible to 20) */
constexpr uint8_t kGameSlotCount          = 4;
constexpr uint8_t kGameSamplesPerCombo    = 8;
constexpr uint16_t kGameStabilitySpanMax  = 100;
constexpr uint8_t kGameStabilityReadCount = 5;

const BoxPieceId kGamePieces[] = {
    BoxPieceId::EMPTY,      BoxPieceId::FORWARD,   BoxPieceId::BACKWARD,
    BoxPieceId::TURN_RIGHT, BoxPieceId::TURN_LEFT, BoxPieceId::LOOP_CALL,
};

/* Active piece count. Bump to 6 when the orange LOOP_CALL piece exists */
constexpr uint8_t kGamePieceCount = 5;

const char *kGamePieceHints[] = {
    "EMPTY (remove piece)",       "FORWARD (green, 3mm)",  "BACKWARD (red, 1mm S)",
    "TURN_RIGHT (yellow, 1mm N)", "TURN_LEFT (blue, 4mm)", "LOOP_CALL (orange, 2mm)",
};

bool gGameActive         = false;
uint8_t gGamePieceIndex  = 0;
uint8_t gGameSlotIndex   = 0;
uint8_t gGameSampleIndex = 0;

/* Change report history for the health cycle: the five ADS slots plus
 * the nRF24 chip state. Filled at the end of each health run and
 * compared against the previous run */
bool gLastProbeState[kAdsCount + 1] = {};
bool gProbeStateValid               = false;

/* Sliced delay so USB CDC commands are polled while the test cycles wait.
 * Rail settle delays keep the plain delay(): the electrical settling must
 * not be interrupted */
void handleUsbCommands();
void recordLabelPoint(uint8_t slotIndex, BoxPieceId piece);
void resetLabeledSlot(uint8_t slotIndex);
void resetLabelPiece(uint8_t slotIndex, BoxPieceId piece);
void runSlotMonitor(uint8_t slotIndex);
void runGapAnalysis();
void recoverAdsModule(const AdsTestEntry *entry);
void emitThresholdHeader();
void printUsbHelp();
bool labelCharToPiece(char ch, BoxPieceId &piece);
const char *labelName(int index);
void startGameMode();
void stopGameMode();
void displayGameInstruction();
bool readSlotWithStability(uint8_t slotIndex, uint16_t &outMean);
bool recordAllSlotsEmpty();
bool attemptGameRecord();
void advanceGameState();
void emitGameThresholds();

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

/* The key list, shown at boot and on demand by '?' */
void printUsbHelp()
{
    Serial.println("   usb: h health, c calibration, s run test, t gaps, x thresholds, ? help");
    Serial.println(
        "   calibration: '<slot><label>' e.g. '5f' (e f b r l L), m = monitor slot, R = reset");
    Serial.println("   game (calibration): 'g' start, SPACE record, 'r' retry step, 'q' quit");
}

void handleUsbCommands()
{
    /* pendingSlot collects a typed slot number that prefixes a label key
     * ('17L'): digits accumulate in a row and the next label letter ends
     * the slot selection; any other key clears the pending digits. With
     * no explicit slot the label reuses the previously addressed slot */
    static uint16_t pendingSlot    = 0;
    static bool digitsStarted      = false;
    static uint8_t lastLabeledSlot = kCalibrationSlotCount; /* none selected */

    while (Serial.available() > 0)
    {
        const int incoming = Serial.read();

        /* While the slot monitor runs, any key is a stop request */
        if (gMonitorActive)
        {
            gMonitorExit  = true;
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        /* Game mode intercepts its own keys; any other key is dropped so
         * it cannot leak into the normal label/digit handler */
        if (gGameActive)
        {
            switch (incoming)
            {
            case ' ': {
                const BoxPieceId currentPiece = kGamePieces[gGamePieceIndex];
                bool ok;
                if (currentPiece == BoxPieceId::EMPTY)
                {
                    ok = recordAllSlotsEmpty();
                }
                else
                {
                    ok = attemptGameRecord();
                }
                if (ok)
                {
                    advanceGameState();
                }
            }
            break;
            case 'r': {
                const uint8_t slot     = gGameSlotIndex;
                const BoxPieceId piece = kGamePieces[gGamePieceIndex];
                if (piece == BoxPieceId::EMPTY)
                {
                    /* Reset all slots for EMPTY */
                    for (uint8_t s = 0; s < kGameSlotCount; ++s)
                    {
                        resetLabelPiece(s, BoxPieceId::EMPTY);
                    }
                    Serial.println("   reset all slots EMPTY cluster, retry");
                }
                else
                {
                    resetLabelPiece(slot, piece);
                    gGameSampleIndex = 0;
                    Serial.print("   reset slot ");
                    Serial.print(static_cast<int>(slot) + 1);
                    Serial.print(" ");
                    Serial.print(labelName(static_cast<int>(piece)));
                    Serial.println(" cluster, retry");
                }
                displayGameInstruction();
            }
            break;
            case 'q':
                stopGameMode();
                break;
            case '?':
                Serial.println("\n   Game mode commands:");
                Serial.println("   SPACE = record, 'r' = retry, 'q' = quit");
                displayGameInstruction();
                break;
            default:
                break;
            }
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        if (incoming >= '0' && incoming <= '9')
        {
            pendingSlot   = static_cast<uint16_t>(pendingSlot * 10 + (incoming - '0'));
            digitsStarted = true;
            continue;
        }

        const char ch    = static_cast<char>(incoming);
        BoxPieceId piece = BoxPieceId::INVALID;
        (void)labelCharToPiece(ch, piece);

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
        case '?':
            Serial.print("mode: ");
            Serial.println(gMode == TestMode::Health ? "health" : "calibration");
            printUsbHelp();
            break;
        case 't':
            runGapAnalysis();
            break;
        case 'x':
            emitThresholdHeader();
            break;
        case 'g':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("game mode works in calibration mode ('c' to switch)");
                break;
            }
            if (gGameActive)
            {
                Serial.println("game mode already active ('q' to stop)");
                break;
            }
            startGameMode();
            break;
        case 'R':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("sample reset works in calibration mode ('c' to switch)");
                break;
            }
            if (digitsStarted)
            {
                if (pendingSlot >= 1 && pendingSlot <= kCalibrationSlotCount)
                {
                    resetLabeledSlot(static_cast<uint8_t>(pendingSlot - 1));
                }
                else
                {
                    Serial.println("slot out of range (1..20)");
                }
            }
            else if (lastLabeledSlot < kCalibrationSlotCount)
            {
                resetLabeledSlot(lastLabeledSlot);
            }
            else
            {
                Serial.println("type '<slot>R' e.g. '5R' (reset slot 5 samples)");
            }
            break;
        case 'm':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("slot monitor works in calibration mode ('c' to switch)");
                break;
            }
            if (digitsStarted)
            {
                if (pendingSlot >= 1 && pendingSlot <= kCalibrationSlotCount)
                {
                    lastLabeledSlot = static_cast<uint8_t>(pendingSlot - 1);
                }
                else
                {
                    Serial.println("slot out of range (1..20)");
                    break;
                }
            }
            else if (lastLabeledSlot >= kCalibrationSlotCount)
            {
                Serial.println("type '<slot>m' e.g. '5m' (monitor slot 5)");
                break;
            }
            runSlotMonitor(lastLabeledSlot);
            break;
        default:
            if (piece == BoxPieceId::INVALID)
            {
                break;
            }
            if (gMode != TestMode::Calibration)
            {
                Serial.println("labels register in calibration mode ('c' to switch)");
                break;
            }
            if (digitsStarted)
            {
                if (pendingSlot >= 1 && pendingSlot <= kCalibrationSlotCount)
                {
                    lastLabeledSlot = static_cast<uint8_t>(pendingSlot - 1);
                }
                else
                {
                    Serial.println("slot out of range (1..20)");
                    break;
                }
            }
            else if (lastLabeledSlot >= kCalibrationSlotCount)
            {
                Serial.println("type '<slot><label>' e.g. '5f' (slot 5 forward)");
                break;
            }
            recordLabelPoint(lastLabeledSlot, piece);
            break;
        }

        digitsStarted = false;
        pendingSlot   = 0;
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

// ==========================================
// PIECE LABELING (calibration mode)
// ==========================================

/* Accumulated samples per (slot, piece): the operator moves a piece into a
 * slot, types '<slot><label>' and the current mean reading lands in that
 * cluster. Across visits, min/max/count map each piece type per sensor.
 * Label indexes match the BoxPieceId enum order EMPTY..LOOP_CALL; INVALID
 * is never labeled */
struct LabelCluster
{
    uint32_t sum;
    uint16_t min;
    uint16_t max;
    uint16_t count;
};

LabelCluster gPieceClusters[kCalibrationSlotCount][6] = {};

const char *labelName(int index)
{
    switch (index)
    {
    case 0:
        return "EMPTY";
    case 1:
        return "FORWARD";
    case 2:
        return "BACKWARD";
    case 3:
        return "TURN_RIGHT";
    case 4:
        return "TURN_LEFT";
    case 5:
        return "LOOP_CALL";
    default:
        return "?";
    }
}

bool labelCharToPiece(char ch, BoxPieceId &piece)
{
    switch (ch)
    {
    case 'e':
        piece = BoxPieceId::EMPTY;
        return true;
    case 'f':
        piece = BoxPieceId::FORWARD;
        return true;
    case 'b':
        piece = BoxPieceId::BACKWARD;
        return true;
    case 'r':
        piece = BoxPieceId::TURN_RIGHT;
        return true;
    case 'l':
        piece = BoxPieceId::TURN_LEFT;
        return true;
    case 'L':
        piece = BoxPieceId::LOOP_CALL;
        return true;
    default:
        return false;
    }
}

void addLabelPoint(uint8_t slotIndex, BoxPieceId piece, uint16_t raw)
{
    LabelCluster &cluster = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];
    if (cluster.count == 0)
    {
        cluster.min   = raw;
        cluster.max   = raw;
        cluster.sum   = raw;
        cluster.count = 1;
        return;
    }
    if (raw < cluster.min)
    {
        cluster.min = raw;
    }
    if (raw > cluster.max)
    {
        cluster.max = raw;
    }
    cluster.sum += raw;
    ++cluster.count;
}

void recordLabelPoint(uint8_t slotIndex, BoxPieceId piece)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    int16_t rawMean = 0;
    bool readOk     = readAdsSingleEndedMean(&entry, channel, rawMean);
    if (!readOk)
    {
        recoverAdsModule(&entry);
        readOk = readAdsSingleEndedMean(&entry, channel, rawMean);
    }
    if (!readOk)
    {
        Serial.print("   slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        Serial.println(" read TIMEOUT/FAIL, point not recorded");
        return;
    }

    const uint16_t raw           = static_cast<uint16_t>(rawMean);
    const LabelCluster &existing = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];

    /* Outlier guard: reject samples far outside the already labeled range,
     * so a bad contact or a misplaced piece cannot silently stretch the
     * cluster. Reset the slot cluster ('R') to re-record from scratch */
    if (existing.count > 0)
    {
        const uint32_t spanBottom =
            existing.min > existing.min / 5 ? existing.min - existing.min / 5 : 0;
        const uint32_t spanTop = existing.max + existing.max / 5;
        if (raw < spanBottom || raw > spanTop)
        {
            Serial.print("   [OUTLIER IGNORED] slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.print(" ");
            Serial.print(labelName(static_cast<int>(piece)));
            Serial.print(" raw=");
            Serial.print(raw);
            Serial.print(" expected.raw ");
            Serial.print(static_cast<int>(spanBottom));
            Serial.print("-");
            Serial.println(static_cast<int>(spanTop));
            return;
        }
    }

    addLabelPoint(slotIndex, piece, raw);
    const LabelCluster &cluster = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];

    Serial.print("   slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" ");
    Serial.print(labelName(static_cast<int>(piece)));
    Serial.print(": raw=");
    Serial.print(raw);
    Serial.print(" n=");
    Serial.println(cluster.count);
}

void resetLabeledSlot(uint8_t slotIndex)
{
    for (int label = 0; label < 6; ++label)
    {
        gPieceClusters[slotIndex][label] = LabelCluster{};
    }
    Serial.print("   slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.println(" samples reset");
}

void resetLabelPiece(uint8_t slotIndex, BoxPieceId piece)
{
    gPieceClusters[slotIndex][static_cast<uint8_t>(piece)] = LabelCluster{};
}

// ==========================================
// GAME MODE — guided piece calibration
// ==========================================

/* Total steps the game will walk through. Each (piece, slot) combo gets
 * kGameSamplesPerCombo recordings */
static uint16_t gameTotalSteps()
{
    return static_cast<uint16_t>(kGamePieceCount) * kGameSlotCount * kGameSamplesPerCombo;
}

/* Current linear step number (1-based) across the whole sequence */
static uint16_t gameCurrentStep()
{
    return static_cast<uint16_t>(gGamePieceIndex) * kGameSlotCount * kGameSamplesPerCombo +
           static_cast<uint16_t>(gGameSlotIndex) * kGameSamplesPerCombo +
           static_cast<uint16_t>(gGameSampleIndex) + 1;
}

void startGameMode()
{
    gGameActive      = true;
    gGamePieceIndex  = 0;
    gGameSlotIndex   = 0;
    gGameSampleIndex = 0;

    Serial.println("\n--- GAME MODE STARTED ---");
    Serial.print("   ");
    Serial.print(gameTotalSteps());
    Serial.print(" steps (");
    Serial.print(static_cast<int>(kGamePieceCount));
    Serial.print(" pieces x ");
    Serial.print(static_cast<int>(kGameSlotCount));
    Serial.print(" slots x ");
    Serial.print(static_cast<int>(kGameSamplesPerCombo));
    Serial.println(" samples)");
    Serial.println("   SPACE = record, 'r' = retry, 'q' = quit");
    displayGameInstruction();
}

void stopGameMode()
{
    gGameActive = false;
    Serial.println("\n--- GAME MODE ENDED ---");
    emitGameThresholds();
}

void displayGameInstruction()
{
    const BoxPieceId piece = kGamePieces[gGamePieceIndex];

    /* EMPTY piece is special: record all slots at once since the board
     * typically starts empty. No need to confirm each slot individually */
    if (piece == BoxPieceId::EMPTY)
    {
        Serial.print("   [");
        Serial.print(gameCurrentStep());
        Serial.print("/");
        Serial.print(gameTotalSteps());
        Serial.print("] ");
        Serial.print(kGamePieceHints[gGamePieceIndex]);
        Serial.println(" for all slots");
        Serial.println("   -> confirm all slots empty, then SPACE");
        return;
    }

    Serial.print("   [");
    Serial.print(gameCurrentStep());
    Serial.print("/");
    Serial.print(gameTotalSteps());
    Serial.print("] slot ");
    Serial.print(static_cast<int>(gGameSlotIndex) + 1);
    Serial.print(", ");
    Serial.print(kGamePieceHints[gGamePieceIndex]);
    Serial.print(" [");
    Serial.print(static_cast<int>(gGameSampleIndex) + 1);
    Serial.print("/");
    Serial.print(static_cast<int>(kGameSamplesPerCombo));
    Serial.println("]");
    Serial.println("   -> place/adjust, wait for stability, then SPACE");
}

/* Read one slot several times and check stability. If stable, write the
 * mean to outMean and return true. Otherwise print the span and return
 * false. Used by both normal recording and bulk EMPTY recording */
bool readSlotWithStability(uint8_t slotIndex, uint16_t &outMean)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    uint16_t minVal = 0xFFFF;
    uint16_t maxVal = 0;
    int32_t sum     = 0;

    for (uint8_t i = 0; i < kGameStabilityReadCount; ++i)
    {
        int16_t raw = 0;
        bool ok     = readAdsSingleEndedMean(&entry, channel, raw);
        if (!ok)
        {
            recoverAdsModule(&entry);
            ok = readAdsSingleEndedMean(&entry, channel, raw);
        }
        if (!ok)
        {
            Serial.print("   slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.println(" read FAIL");
            return false;
        }
        const uint16_t uval = static_cast<uint16_t>(raw);
        if (uval < minVal)
        {
            minVal = uval;
        }
        if (uval > maxVal)
        {
            maxVal = uval;
        }
        sum += raw;
    }

    const uint16_t span = maxVal - minVal;
    if (span > kGameStabilitySpanMax)
    {
        Serial.print("   slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        Serial.print(" unstable (span=");
        Serial.print(span);
        Serial.print(", min=");
        Serial.print(minVal);
        Serial.print(", max=");
        Serial.print(maxVal);
        Serial.println(")");
        return false;
    }

    outMean = static_cast<uint16_t>(sum / kGameStabilityReadCount);
    Serial.print("   slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" raw=");
    Serial.print(outMean);
    Serial.print(" (span=");
    Serial.print(span);
    Serial.println(")");
    return true;
}

/* Record EMPTY for all slots at once. Used when the board starts empty
 * to avoid tedious per-slot confirmation */
bool recordAllSlotsEmpty()
{
    Serial.println("   recording EMPTY for all slots...");

    for (uint8_t slot = 0; slot < kGameSlotCount; ++slot)
    {
        uint16_t mean = 0;
        if (!readSlotWithStability(slot, mean))
        {
            Serial.println("   retry SPACE");
            return false;
        }
        addLabelPoint(slot, BoxPieceId::EMPTY, mean);
    }

    return true;
}

/* Read the slot several times and check the span is within the stability
 * threshold. If stable, record one point via addLabelPoint and return
 * true. Otherwise print the span and return false. EMPTY is handled
 * specially by recordAllSlotsEmpty() so this function assumes non-EMPTY */
bool attemptGameRecord()
{
    const uint8_t slotIndex   = gGameSlotIndex;
    const BoxPieceId piece    = kGamePieces[gGamePieceIndex];
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    uint16_t minVal = 0xFFFF;
    uint16_t maxVal = 0;
    int32_t sum     = 0;

    for (uint8_t i = 0; i < kGameStabilityReadCount; ++i)
    {
        int16_t raw = 0;
        bool ok     = readAdsSingleEndedMean(&entry, channel, raw);
        if (!ok)
        {
            recoverAdsModule(&entry);
            ok = readAdsSingleEndedMean(&entry, channel, raw);
        }
        if (!ok)
        {
            Serial.println("   read FAIL, press SPACE again");
            return false;
        }
        const uint16_t uval = static_cast<uint16_t>(raw);
        if (uval < minVal)
        {
            minVal = uval;
        }
        if (uval > maxVal)
        {
            maxVal = uval;
        }
        sum += raw;
    }

    const uint16_t span = maxVal - minVal;
    if (span > kGameStabilitySpanMax)
    {
        Serial.print("   unstable (span=");
        Serial.print(span);
        Serial.print(", min=");
        Serial.print(minVal);
        Serial.print(", max=");
        Serial.print(maxVal);
        Serial.println("), press SPACE again");
        return false;
    }

    const uint16_t mean = static_cast<uint16_t>(sum / kGameStabilityReadCount);

    /* Outlier guard: same ±20% logic as recordLabelPoint, so a misplaced
     * piece cannot silently stretch the cluster */
    const LabelCluster &existing = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];
    if (existing.count > 0)
    {
        const uint32_t spanBottom =
            existing.min > existing.min / 5 ? existing.min - existing.min / 5 : 0;
        const uint32_t spanTop = existing.max + existing.max / 5;
        if (mean < spanBottom || mean > spanTop)
        {
            Serial.print("   [OUTLIER IGNORED] raw=");
            Serial.print(mean);
            Serial.print(" expected ");
            Serial.print(static_cast<int>(spanBottom));
            Serial.print("-");
            Serial.println(static_cast<int>(spanTop));
            return false;
        }
    }

    addLabelPoint(slotIndex, piece, mean);

    Serial.print("   recorded raw=");
    Serial.print(mean);
    Serial.print(" (span=");
    Serial.print(span);
    Serial.print(", n=");
    Serial.print(existing.count + 1);
    Serial.println(")");
    return true;
}

void advanceGameState()
{
    const BoxPieceId currentPiece = kGamePieces[gGamePieceIndex];

    /* EMPTY is recorded in bulk for all slots at once, so skip all
     * EMPTY steps and jump directly to the next piece */
    if (currentPiece == BoxPieceId::EMPTY)
    {
        gGameSampleIndex = 0;
        gGameSlotIndex   = 0;
        ++gGamePieceIndex;
        if (gGamePieceIndex >= kGamePieceCount)
        {
            stopGameMode();
            return;
        }
        displayGameInstruction();
        return;
    }

    ++gGameSampleIndex;
    if (gGameSampleIndex >= kGameSamplesPerCombo)
    {
        gGameSampleIndex = 0;
        ++gGameSlotIndex;
        if (gGameSlotIndex >= kGameSlotCount)
        {
            gGameSlotIndex = 0;
            ++gGamePieceIndex;
            if (gGamePieceIndex >= kGamePieceCount)
            {
                stopGameMode();
                return;
            }
            Serial.println();
            Serial.print("=== NEW PIECE: ");
            Serial.print(kGamePieceHints[gGamePieceIndex]);
            Serial.println(" ===");
        }
        else
        {
            Serial.println();
            Serial.print("=== MOVE PIECE TO SLOT ");
            Serial.print(static_cast<int>(gGameSlotIndex) + 1);
            Serial.println(" ===");
        }
    }
    displayGameInstruction();
}

/* Compute per-piece aggregated stats across game slots, sort by mean,
 * place thresholds at midpoints between adjacent classes and emit
 * #define blocks in the same format as piece_thresholds.h */
void emitGameThresholds()
{
    /* Aggregate min/max/sum/count per piece across the kGameSlotCount slots */
    struct PieceRange
    {
        BoxPieceId piece;
        uint16_t gmin;
        uint16_t gmax;
        uint32_t gsum;
        uint32_t gcount;
    };
    PieceRange ranges[kGamePieceCount];

    for (uint8_t p = 0; p < kGamePieceCount; ++p)
    {
        PieceRange &r = ranges[p];
        r.piece       = kGamePieces[p];
        r.gmin        = 0xFFFF;
        r.gmax        = 0;
        r.gsum        = 0;
        r.gcount      = 0;

        for (uint8_t s = 0; s < kGameSlotCount; ++s)
        {
            const LabelCluster &cl = gPieceClusters[s][static_cast<uint8_t>(r.piece)];
            if (cl.count == 0)
            {
                continue;
            }
            if (cl.min < r.gmin)
            {
                r.gmin = cl.min;
            }
            if (cl.max > r.gmax)
            {
                r.gmax = cl.max;
            }
            r.gsum += cl.sum;
            r.gcount += cl.count;
        }
    }

    /* Print per-piece ranges */
    Serial.println("\n--- PER-PIECE RANGES (game data) ---");
    for (uint8_t p = 0; p < kGamePieceCount; ++p)
    {
        const PieceRange &r = ranges[p];
        Serial.print("   ");
        Serial.print(labelName(static_cast<int>(r.piece)));
        if (r.gcount == 0)
        {
            Serial.println("  : NO DATA");
            continue;
        }
        Serial.print("  : min=");
        Serial.print(r.gmin);
        Serial.print(" max=");
        Serial.print(r.gmax);
        Serial.print(" mean=");
        Serial.println(static_cast<int32_t>(r.gsum / r.gcount));
    }

    /* Sort ranges by mean (insertion sort, 5-6 elements) */
    for (uint8_t i = 1; i < kGamePieceCount; ++i)
    {
        PieceRange tmp  = ranges[i];
        int32_t tmpMean = tmp.gcount > 0 ? static_cast<int32_t>(tmp.gsum / tmp.gcount) :
                                           static_cast<int32_t>(tmp.gmin);
        uint8_t j       = i;
        while (j > 0)
        {
            const PieceRange &prev = ranges[j - 1];
            int32_t prevMean = prev.gcount > 0 ? static_cast<int32_t>(prev.gsum / prev.gcount) :
                                                 static_cast<int32_t>(prev.gmin);
            if (prevMean > tmpMean)
            {
                ranges[j] = ranges[j - 1];
                --j;
            }
            else
            {
                break;
            }
        }
        ranges[j] = tmp;
    }

    /* Print sorted order + midpoints */
    Serial.println("\n--- SORTED ORDER (ascending mean) ---");
    uint16_t thresholds[kGamePieceCount];
    for (uint8_t i = 0; i < kGamePieceCount; ++i)
    {
        const PieceRange &r = ranges[i];
        const int32_t mean  = r.gcount > 0 ? static_cast<int32_t>(r.gsum / r.gcount) : 0;
        Serial.print("   ");
        Serial.print(static_cast<int>(i));
        Serial.print(". ");
        Serial.print(labelName(static_cast<int>(r.piece)));
        Serial.print("  mean=");
        Serial.print(mean);
        Serial.print(" [");
        Serial.print(r.gmin);
        Serial.print("..");
        Serial.print(r.gmax);
        Serial.println("]");

        if (i == 0)
        {
            thresholds[i] = r.gmin;
        }
        else
        {
            const PieceRange &below = ranges[i - 1];
            thresholds[i]           = static_cast<uint16_t>(
                (static_cast<uint32_t>(below.gmax) + static_cast<uint32_t>(r.gmin)) / 2);
        }
    }

    /* Print gap analysis */
    Serial.println("\n--- GAPS (between adjacent sorted classes) ---");
    for (uint8_t i = 1; i < kGamePieceCount; ++i)
    {
        const PieceRange &below = ranges[i - 1];
        const PieceRange &above = ranges[i];
        Serial.print("   ");
        Serial.print(labelName(static_cast<int>(below.piece)));
        Serial.print(" -> ");
        Serial.print(labelName(static_cast<int>(above.piece)));
        Serial.print(" : gap=");
        Serial.print(static_cast<int>(above.gmin) - static_cast<int>(below.gmax));
        Serial.print(" threshold=");
        Serial.println(thresholds[i]);
    }

    /* Emit #define block in the piece_thresholds.h format. Map each
     * threshold to the macro name that corresponds to the piece ABOVE
     * the boundary. The lowest piece's threshold is its own start */
    Serial.println("\n--- GENERATED piece_thresholds.h ---");
    Serial.println("#pragma once");
    Serial.println();
    Serial.println("#include <stdint.h>");
    Serial.println();

    /* Map BoxPieceId -> macro suffix. Uses the same names as the
     * existing header so the output can be pasted directly */
    const char *macroSuffix[kGamePieceCount];
    for (uint8_t i = 0; i < kGamePieceCount; ++i)
    {
        macroSuffix[i] = labelName(static_cast<int>(ranges[i].piece));
    }

    for (uint8_t i = 0; i < kGamePieceCount; ++i)
    {
        const uint16_t value = thresholds[i];
        /* Build the macro name: GASPETTO_ADC_THRESHOLD_<PIECE>_START.
         * The label names use the same wording as the existing macros
         * (FORWARD, BACKWARD, TURN_RIGHT, TURN_LEFT, LOOP_CALL). EMPTY
         * gets a new EMPTY_START for the restructured decode table */
        Serial.print("#ifndef GASPETTO_ADC_THRESHOLD_");
        Serial.print(macroSuffix[i]);
        Serial.println("_START");
        Serial.print("#define GASPETTO_ADC_THRESHOLD_");
        Serial.print(macroSuffix[i]);
        Serial.print("_START ");
        Serial.println(value);
        Serial.println("#endif");
        Serial.println();
    }

    Serial.print("#define GASPETTO_ADC_THRESHOLD_MAX ");
    /* MAX = one full-scale margin above the highest piece max. Use the
     * highest observed max + 10% headroom, capped at 26400 (ADS1115
     * 16-bit unsigned full scale) */
    const PieceRange &top = ranges[kGamePieceCount - 1];
    uint32_t maxCeil = top.gcount > 0 ? static_cast<uint32_t>(top.gmax) + top.gmax / 10 : 26400;
    if (maxCeil > 26400)
    {
        maxCeil = 26400;
    }
    Serial.println(maxCeil);
}

/* Continuous readout of one slot: ~5 mean reads per second with running
 * min/max. Any usb key stops it. Clamp detection: a max pinned at the
 * top of the scale (near 26400 on the 3V3 rail) means the hall sensor
 * output is saturated, not that the piece reads high */
void runSlotMonitor(uint8_t slotIndex)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    Serial.print("--- MONITOR slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" (");
    Serial.print(entry.busName);
    Serial.println("), any key to stop ---");
    if (!gCalibrationRailsOn)
    {
        Serial.println("   NOTE: 3V3 rail is OFF, readings will be garbage");
    }

    uint16_t minRaw = 0xFFFF;
    uint16_t maxRaw = 0;
    uint32_t count  = 0;
    uint64_t sum    = 0;

    gMonitorActive = true;
    gMonitorExit   = false;
    while (!gMonitorExit)
    {
        int16_t rawMean = 0;
        bool readOk     = readAdsSingleEndedMean(&entry, channel, rawMean);
        if (!readOk)
        {
            Serial.println("   read TIMEOUT/FAIL, recovering module");
            recoverAdsModule(&entry);
            readOk = readAdsSingleEndedMean(&entry, channel, rawMean);
        }
        if (readOk)
        {
            const uint16_t raw = static_cast<uint16_t>(rawMean);
            if (raw < minRaw)
            {
                minRaw = raw;
            }
            if (raw > maxRaw)
            {
                maxRaw = raw;
            }
            sum += raw;
            ++count;
        }

        /* Group the readouts by four so the addressed ADS module is
         * always visible: one group header per four readings */
        if (readOk && count % 4 == 1)
        {
            Serial.print("   [ADS module ");
            Serial.print(static_cast<int>(slotIndex / 4) + 1);
            Serial.print("/5: ");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.print(", slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.print(" = channel ");
            Serial.print(static_cast<int>(channel) + 1);
            Serial.println("/4]");
        }

        Serial.print("   slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        Serial.print(": raw=");
        if (readOk)
        {
            Serial.print(rawMean);
        }
        else
        {
            Serial.print("----");
        }
        Serial.print(" min=");
        if (count > 0)
        {
            Serial.print(static_cast<int>(minRaw));
            Serial.print(" max=");
            Serial.println(maxRaw);
        }
        else
        {
            Serial.println("---- max=----");
        }

        /* The parser only sets gMonitorExit here, so the sampling
         * period doubles as the usb response time */
        delayWithUsb(200);
    }
    gMonitorActive = false;

    Serial.println("--- MONITOR stopped ---");
    if (count > 0)
    {
        Serial.print("   samples n=");
        Serial.print(static_cast<int>(count));
        Serial.print(" mean=");
        Serial.print(static_cast<int>(sum / count));
        Serial.print(" min=");
        Serial.print(static_cast<int>(minRaw));
        Serial.print(" max=");
        Serial.print(static_cast<int>(maxRaw));
        Serial.print(" span=");
        Serial.println(static_cast<int>(maxRaw - minRaw));
    }
}

/* One labeled piece type seen across all slots, min/max in raw counts */
struct LabelRange
{
    int label;
    uint32_t min;
    uint32_t max;
    uint32_t count;
};

/* Threshold macro name for a piece type, nullptr when it has none */
const char *thresholdStartName(int label)
{
    switch (label)
    {
    case 0:
        return "GASPETTO_ADC_THRESHOLD_EMPTY_START";
    case 1:
        return "GASPETTO_ADC_THRESHOLD_FORWARD_START";
    case 2:
        return "GASPETTO_ADC_THRESHOLD_BACKWARD_START";
    case 3:
        return "GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START";
    case 4:
        return "GASPETTO_ADC_THRESHOLD_TURN_LEFT_START";
    case 5:
        return "GASPETTO_ADC_THRESHOLD_LOOP_START";
    default:
        return nullptr;
    }
}

/* Aggregate every labeled cluster by piece type, lowest first */
void collectSortedPieceRanges(LabelRange *ranges, uint8_t &written)
{
    written = 0;
    for (int label = 0; label < 6; ++label)
    {
        uint32_t min   = 0xFFFFFFFFUL;
        uint32_t max   = 0;
        uint32_t count = 0;
        for (uint8_t slotIndex = 0; slotIndex < kCalibrationSlotCount; ++slotIndex)
        {
            const LabelCluster &cluster = gPieceClusters[slotIndex][label];
            if (cluster.count == 0)
            {
                continue;
            }
            count += cluster.count;
            if (cluster.min < min)
            {
                min = cluster.min;
            }
            if (cluster.max > max)
            {
                max = cluster.max;
            }
        }
        if (count > 0)
        {
            ranges[written].label = label;
            ranges[written].min   = min;
            ranges[written].max   = max;
            ranges[written].count = count;
            ++written;
        }
    }

    /* lowest first */
    for (uint8_t i = 0; i < written; ++i)
    {
        for (uint8_t j = i + 1; j < written; ++j)
        {
            if (ranges[j].min < ranges[i].min)
            {
                const LabelRange swap = ranges[i];
                ranges[i]             = ranges[j];
                ranges[j]             = swap;
            }
        }
    }
}

void runGapAnalysis()
{
    LabelRange ranges[6];
    uint8_t written = 0;
    collectSortedPieceRanges(ranges, written);
    Serial.println("--- PIECE THRESHOLD ANALYSIS ---");
    if (written == 0)
    {
        Serial.println("   no labeled samples; enter calibration ('c') then '<slot><label>'");
        return;
    }
    for (uint8_t i = 0; i < written; ++i)
    {
        Serial.print("   ");
        Serial.print(labelName(ranges[i].label));
        Serial.print(": raw ");
        Serial.print(static_cast<int>(ranges[i].min));
        Serial.print("-");
        Serial.print(static_cast<int>(ranges[i].max));
        Serial.print(", n=");
        Serial.print(static_cast<int>(ranges[i].count));
        Serial.println();
    }

    for (uint8_t i = 0; i + 1 < written; ++i)
    {
        const LabelRange &lower = ranges[i];
        const LabelRange &upper = ranges[i + 1];
        if (lower.max >= upper.min)
        {
            Serial.print("   [AMBIGUOUS] ");
            Serial.print(labelName(lower.label));
            Serial.print(" and ");
            Serial.print(labelName(upper.label));
            Serial.println(" ranges overlap; separate them before trusting thresholds");
            continue;
        }

        const uint32_t suggested = (lower.max + upper.min) / 2;
        Serial.print("   #define ");
        Serial.print(thresholdStartName(upper.label));
        Serial.print(" ");
        Serial.println(static_cast<int>(suggested));
    }
    Serial.println("   note: top of highest labeled piece informs LOOP_END");
}

void emitThresholdHeader()
{
    LabelRange ranges[6];
    uint8_t written = 0;
    collectSortedPieceRanges(ranges, written);

    Serial.println("--- GENERATED piece_thresholds.h BLOCK (raw ADS1115 counts) ---");
    if (written == 0)
    {
        Serial.println("   no labeled samples; enter calibration ('c') then '<slot><label>'");
        return;
    }

    Serial.println("#pragma once");
    Serial.println();
    Serial.println("/* Generated by arduino_box_hw_test from labeled piece samples */");

    /* Neighborhoods drive the defines: the gap between two adjacent
     * labeled piece types becomes the upper piece's *_START */
    for (uint8_t i = 0; i + 1 < written; ++i)
    {
        const LabelRange &lower = ranges[i];
        const LabelRange &upper = ranges[i + 1];
        const char *name        = thresholdStartName(upper.label);
        if (lower.max >= upper.min)
        {
            Serial.print("/* AMBIGUOUS: cannot derive ");
            Serial.print(name);
            Serial.print(" (");
            Serial.print(labelName(lower.label));
            Serial.print(" overlaps ");
            Serial.print(labelName(upper.label));
            Serial.println(") */");
            continue;
        }

        const uint32_t suggested = (lower.max + upper.min) / 2;
        Serial.print("#define ");
        Serial.print(name);
        Serial.print(" ");
        Serial.print(static_cast<int>(suggested));
        Serial.print("  /* gap raw ");
        Serial.print(static_cast<int>(lower.max));
        Serial.print("-");
        Serial.print(static_cast<int>(upper.min));
        Serial.print(", n=");
        Serial.print(static_cast<int>(lower.count + upper.count));
        Serial.println(" */");
    }

    /* LOOP_END just above the top of the highest labeled piece */
    const LabelRange &top = ranges[written - 1];
    Serial.print("#define GASPETTO_ADC_THRESHOLD_LOOP_END ");
    Serial.print(static_cast<int>(top.max + 1));
    Serial.print("  /* above ");
    Serial.print(labelName(top.label));
    Serial.print(" top raw ");
    Serial.print(static_cast<int>(top.max));
    Serial.println(" */");
    Serial.println("#define GASPETTO_ADC_THRESHOLD_MAX 32767");
    Serial.println("--- END BLOCK: paste over the defines of piece_thresholds.h ---");
}

const char *pieceLabelForRaw(int16_t raw)
{
    if (raw < 0)
    {
        return "INVALID";
    }
    if (raw < GASPETTO_ADC_THRESHOLD_TURN_LEFT_START)
    {
        return "INVALID";
    }
    if (raw < GASPETTO_ADC_THRESHOLD_BACKWARD_START)
    {
        return "TURN_LEFT";
    }
    if (raw < GASPETTO_ADC_THRESHOLD_EMPTY_START)
    {
        return "BACKWARD";
    }
    if (raw < GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START)
    {
        return "EMPTY";
    }
    if (raw < GASPETTO_ADC_THRESHOLD_FORWARD_START)
    {
        return "TURN_RIGHT";
    }
    if (raw < GASPETTO_ADC_THRESHOLD_LOOP_START)
    {
        return "FORWARD";
    }
    if (raw <= GASPETTO_ADC_THRESHOLD_LOOP_END)
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
    Serial.print(GASPETTO_ADC_THRESHOLD_FORWARD_START);
    Serial.print(" BACK:");
    Serial.print(GASPETTO_ADC_THRESHOLD_BACKWARD_START);
    Serial.print(" RIGHT:");
    Serial.print(GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START);
    Serial.print(" LEFT:");
    Serial.print(GASPETTO_ADC_THRESHOLD_TURN_LEFT_START);
    Serial.print(" LOOP:");
    Serial.print(GASPETTO_ADC_THRESHOLD_LOOP_START);
    Serial.print(" LOOPEND:");
    Serial.print(GASPETTO_ADC_THRESHOLD_LOOP_END);
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

    Serial.println("--- LABELED PIECE SAMPLES ---");
    for (uint8_t slotIndex = 0; slotIndex < kCalibrationSlotCount; ++slotIndex)
    {
        for (int label = 0; label < 6; ++label)
        {
            const LabelCluster &cluster = gPieceClusters[slotIndex][label];
            if (cluster.count == 0)
            {
                continue;
            }
            Serial.print("   slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.print(" ");
            Serial.print(labelName(label));
            Serial.print(": n=");
            Serial.print(cluster.count);
            Serial.print(" raw ");
            Serial.print(cluster.min);
            Serial.print("-");
            Serial.print(cluster.max);
            Serial.print(" mean=");
            Serial.println(static_cast<int>(cluster.sum / cluster.count));
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
    printUsbHelp();
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

        /* Game mode: no periodic scan, just USB polling between steps */
        if (gGameActive)
        {
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

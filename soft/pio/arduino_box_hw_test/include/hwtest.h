#pragma once
/* Internal plumbing of the hardware test firmware: the types, board
 * constants and functions each module exposes to the others. Not a
 * library API: everything here is specific to this bench application */
#include "CommandPacket.h"
#include "piece_thresholds.h"
#include "pin_definitions.h"

#include <Arduino.h>
#include <RF24.h>
#include <Wire.h>

#ifdef TEST_LED_ANIMATIONS
#include <Adafruit_NeoPixel.h>
#endif

/* Piece thresholds come from the shared piece_thresholds.h with their
 * static asserts: same numbers as the production firmware */

constexpr uint32_t kI2c1ClockHz = 100000;

#ifndef TEST_ADS1115_MEAN_SAMPLES
#define TEST_ADS1115_MEAN_SAMPLES 10
#endif

static_assert(TEST_ADS1115_MEAN_SAMPLES > 0, "TEST_ADS1115_MEAN_SAMPLES must be >= 1");

constexpr uint8_t kAdsMeanSamples = TEST_ADS1115_MEAN_SAMPLES;

struct CalibrationStats
{
    bool initialized;
    int16_t minRaw;
    int16_t maxRaw;
};

/* Runtime test modes, switched over the USB CDC port. Health runs the
 * connectivity cycle each pass; Calibration keeps the 3V3 sensor rail
 * powered and watches piece ADC values continuously */
enum class TestMode : uint8_t
{
    Health,
    Calibration,
};

struct AdsTestEntry
{
    bool useBitBang; // true = use bit-banging, false = use TwoWire
    TwoWire *wire;   // only used if useBitBang is false
    uint8_t address;
    const char *busName;
    const char *addrNote;
};

/* Five ADS1115 modules of four slots each, fixed by the board wiring;
 * the shared per-slot threshold table covers the same slots */
constexpr uint8_t kAdsCount             = 5;
constexpr uint8_t kCalibrationSlotCount = kAdsCount * 4;

void handleUsbCommands();

/* Sliced delay so USB CDC commands are polled while the test cycles wait.
 * Rail settle delays keep the plain delay(): the electrical settling must
 * not be interrupted */

inline void delayWithUsb(uint32_t totalMs)
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

// ==========================================
// clusters.cpp - label storage
// ==========================================
/* Accumulated samples per (slot, piece): the operator places a piece in a
 * slot, the game records the current mean reading and, across visits,
 * min/max/count map each piece type per sensor. Label indexes match the
 * BoxPieceId enum order EMPTY..LOOP_CALL; INVALID is never labeled */
struct LabelCluster
{
    uint32_t sum;
    uint16_t min;
    uint16_t max;
    uint16_t count;
};

extern LabelCluster gPieceClusters[kCalibrationSlotCount][6];

const char *labelName(int index);
void addLabelPoint(uint8_t slotIndex, BoxPieceId piece, uint16_t raw);
void resetLabeledSlot(uint8_t slotIndex);
void resetLabelPiece(uint8_t slotIndex, BoxPieceId piece);

// ==========================================
// game.cpp - guided campaign walk and row generation
// ==========================================
/* Slots covered by the guided walk (the soldered main zone, up to 20) */
constexpr uint8_t kGameSlotCount = 20;

extern const BoxPieceId kGamePieces[];
extern bool gGameActive;
extern bool gGameAuto;
extern bool gGameOutlierWarned;
extern uint8_t gGamePieceIndex;
extern uint8_t gGameSlotIndex;
extern uint8_t gGameSampleIndex;

/* Outcome of one sample attempt in game mode, so the manual SPACE handler
 * and the auto-capture poll can share the same read/guard logic but react
 * differently (print vs stay silent, advance vs keep waiting) */
enum class GameSampleResult : uint8_t
{
    Recorded,       /* sample added to the cluster */
    Unstable,       /* span too wide, nothing recorded */
    Outlier,        /* stable but outside the cluster guard, nothing recorded */
    ReadFail,       /* ADS read failed after recovery, nothing recorded */
    PieceNotPlaced, /* stable but the slot still reads empty: piece not placed yet */
};

void startGameMode();
void startGameScoped(uint8_t slotIndex, int pieceIndex);
void stopGameMode();
void displayGameInstruction();
bool recordCurrentEmptyStep();
void retryGameStep();
int gamePieceForChar(char ch);
GameSampleResult recordGameSample(uint8_t slotIndex, BoxPieceId piece, bool verbose);
void pollGameAutoCapture();
void advanceGameState();
void emitGameThresholds();

// ==========================================
// scan.cpp - calibration scan and slot monitor
// ==========================================
extern bool gMonitorActive;
extern bool gMonitorExit;

/* Frozen slot monitor state: the shell raises gFreezeMeasure on SPACE
 * (unless a window is already running) and gFreezeExit on any other key */
extern bool gFreezeActive;
extern bool gFreezeMeasure;
extern bool gFreezeMeasuring;
extern bool gFreezeExit;

void runSlotMonitor(uint8_t slotIndex);
void runFrozenSlotMonitor(uint8_t slotIndex);
void runPieceCalibrationScan();

/* Mode state, defined by main.cpp and read by nearly every module */
extern TestMode gMode;
extern bool gCalibrationRailsOn;
extern bool gRunTestRequested;

// ==========================================
// shell.cpp - usb key dispatch, echo and help
// ==========================================
void enterMode(TestMode mode);
void printUsbHelp();

// ==========================================
// ads.cpp - I2C bus layer, ADS1115 reads, recovery and probe history
// ==========================================
extern TwoWire &i2c1;
extern RF24 radio;
extern AdsTestEntry kAdsTests[kAdsCount];
extern CalibrationStats gCalibrationStats[kCalibrationSlotCount];
extern uint32_t gCalibrationScanCounter;

void probeI2CDevices();
void reportProbeChanges();
void resetCalibrationStats();
void updateCalibrationStats(uint8_t slotIndex, int16_t raw);
bool isI2c3Entry(const AdsTestEntry *entry);
void recoverI2c3Bus();
void recoverAdsModule(const AdsTestEntry *entry);
bool readAdsConfigRegister(const AdsTestEntry *entry, uint16_t &config);
bool readAdsSingleEndedMean(const AdsTestEntry *entry, uint8_t channel, int16_t &meanRaw);

// ==========================================
// leds.cpp - APA106 status LEDs and animations
// ==========================================
#ifdef TEST_LED_ANIMATIONS
extern Adafruit_NeoPixel leds;

void blackoutLeds();
void setOneLed(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void setTwoLeds(uint8_t ledA, uint8_t rA, uint8_t gA, uint8_t bA, uint8_t ledB, uint8_t rB,
                uint8_t gB, uint8_t bB);
void runScanAnimationTest();
void runSuccessAnimationTest();
void runBuildErrorAnimationTest();
void runEmptyBoardAnimationTest();
void runRfErrorAnimationTest();
void runPowerOnCycleAnimation();
void runPowerOffCycleAnimation();
#endif /* TEST_LED_ANIMATIONS */

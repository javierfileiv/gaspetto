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

/* Piece thresholds come from the shared piece_thresholds.h with their
 * static asserts: same numbers as the production firmware */

constexpr uint32_t kI2c1ClockHz = 100000;

#ifndef TEST_ADS1115_MEAN_SAMPLES
#define TEST_ADS1115_MEAN_SAMPLES 10
#endif

static_assert(TEST_ADS1115_MEAN_SAMPLES > 0, "TEST_ADS1115_MEAN_SAMPLES must be >= 1");

constexpr uint8_t kAdsMeanSamples = TEST_ADS1115_MEAN_SAMPLES;

/* Settle time after the sensor rail changes state: a hall sensor answers in
 * microseconds, so 250 ms covers the rail's own RC plus margin. Every read
 * raises the rail for its duration and drops it again (see
 * readAdsSingleEndedMean), so this is also the pause between two readings */
constexpr uint16_t kRailSettleDelayMs = 250;

/* Raise the sensor rail when it is down and report whether this call did it,
 * so the caller drops it again only in that case: a rail someone else holds
 * (the health cycle, an enclosing scan) is never taken away. Every I2C access
 * to the ADS chips needs the rail, and it must be held for the whole
 * operation - probe, reads and recovery - not only around one reading */
bool sensorRailAcquire();
void sensorRailRelease(bool acquired);

struct CalibrationStats
{
    bool initialized;
    int16_t minRaw;
    int16_t maxRaw;
};

/* Runtime test modes, switched over the USB CDC port. Health runs the
 * connectivity cycle each pass; Calibration watches piece ADC values, with
 * the sensor rail raised per reading rather than held on for the session */
enum class TestMode : uint8_t
{
    Health,
    Calibration,
};

struct AdsTestEntry
{
    bool useBitBang; /* true = use bit-banging, false = use TwoWire */
    TwoWire *wire;   /* only used if useBitBang is false */
    uint8_t address;
    const char *busName;
    const char *addrNote;
};

/* Five ADS1115 modules of four slots each, fixed by the board wiring;
 * the shared per-slot threshold table covers the same slots */
constexpr uint8_t kAdsCount             = 5;
constexpr uint8_t kCalibrationSlotCount = kAdsCount * 4;

/* The packet's slot count and the number of sensors this firmware reads
 * describe the same board. The radio send test walks the packet slots and uses
 * each as an index into kAdsTests, so the two must stay equal, or the loop
 * would run past the table. Refuse to build rather than overrun at runtime */
static_assert(BOX_TOTAL_SLOTS == kCalibrationSlotCount, "packet slots must match the ADS scan");

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

/* ========================================== */
/* scan.cpp - label names */
/* ========================================== */
/* Names a BoxPieceId value (0..6): the five pieces, plus the INVALID name
 * the decode returns for uncalibrated slots */
const char *labelName(int index);

/* ========================================== */
/* jig.cpp - piece census and slot gain campaign */
/* ========================================== */
/* Measures every piece once in one slot, then one piece across the board to
 * get each slot's gain, and derives the per-slot rows from the two. No
 * keypress during the campaign: the reading says when a piece is in */
extern bool gJigActive;

void startJigMode();
void stopJigMode();
void clearJigCaptures();
void forgetJigSlot(uint8_t slot);
void printJigHelp();
void runJigKey(char key);
void pollJig();
void dumpJigCaptures();

/* ========================================== */
/* scan.cpp - calibration scan and slot monitor */
/* ========================================== */
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

/* The one piece decode: a raw reading against this slot's compiled row.
 * Every consumer (monitors, scan, radio send) goes through it */
BoxPieceId pieceForRaw(uint8_t slotIndex, int16_t raw);

/* Mode state, defined by main.cpp and read by nearly every module */
extern TestMode gMode;
extern bool gCalibrationRailsOn;
extern bool gRunTestRequested;
extern bool gLedMenuActive;
extern bool gRadioMenuActive;

/* main.cpp: deep sleep (STOP) test driven by the "b" shell key */
void runDeepSleepTest();

/* main.cpp: scan -> build -> transmit test driven by the "p" shell key */
void runSendPiecesTest();

/* main.cpp: listen to the car stream on _box_, the "e" shell key */
void runCarStreamTest();

/* main.cpp: radio menu driven by the "t" shell key */
void startRadioMenu();
void runRadioMenuKey(char key);

/* ========================================== */
/* shell.cpp - usb key dispatch, echo and help */
/* ========================================== */
void enterMode(TestMode mode);
void printUsbHelp();

/* ========================================== */
/* ads.cpp - I2C bus layer, ADS1115 reads, recovery and probe history */
/* ========================================== */
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
void rearmAdsModule(const AdsTestEntry *entry);
bool readAdsConfigRegister(const AdsTestEntry *entry, uint16_t &config);
bool readAdsSingleEndedMean(const AdsTestEntry *entry, uint8_t channel, int16_t &meanRaw);

/* ========================================== */
/* leds.cpp - APA106 status LEDs and animations */
/* ========================================== */
/* The chain is driven by the FastLED library (dedicated APA106
 * controller, RGB wire order measured on this batch) */

/* Pixels on the chain: each APA106 regenerates the signal for the next
 * one, so a pixel soldered later lights as soon as it is wired after the
 * previous DOUT. Override without editing: -D APA_LED_COUNT=n */
#ifndef APA_LED_COUNT
#define APA_LED_COUNT 3
#endif
constexpr uint16_t kApaPixelCount = APA_LED_COUNT;

/* Global output level handed to FastLED (255 = full); override with
 * -D APA_LED_INTENSITY=n when full scale is too harsh on the bench */
#ifndef APA_LED_INTENSITY
#define APA_LED_INTENSITY 80
#endif
constexpr uint8_t kApaIntensity = APA_LED_INTENSITY;

/* Prepare the data pin and send one dark frame; call it AFTER the 5V
 * rail is up and settled (the APA106 ignores frames while unpowered) */
void ledsBegin();
void blackoutLeds();
void setOneLed(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void setTwoLeds(uint8_t ledA, uint8_t rA, uint8_t gA, uint8_t bA, uint8_t ledB, uint8_t rB,
                uint8_t gB, uint8_t bB);
void runLedWalkTest();
void runScanAnimationTest();
void runSuccessAnimationTest();
void runBuildErrorAnimationTest();
void runEmptyBoardAnimationTest();
void runRfErrorAnimationTest();
void runChannelOrderTest(uint8_t led);
void runPowerOnCycleAnimation();
void runPowerOffCycleAnimation();
void startLedTestMenu();
void runLedTestKey(char key);

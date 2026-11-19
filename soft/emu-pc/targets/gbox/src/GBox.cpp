#include "GBox.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "CommandPacket.h"
#include "Context.h"
#include "Events.h"
#include "PieceProgram.h"
#include "RadioController.h"
#include "led_strip.h"
#include "piece_decode.h"
#include "pin_definitions.h"

#include <Adafruit_ADS1X15.h>
#include <array>
#include <cstdint>
#ifdef ARDUINO
#include "BitBangWire.h"
#include "i2c3_bitbang.h"
#include "stm32f4xx_hal.h"

#endif
#include <Wire.h>

namespace
{
#ifndef GASPETTO_I2C_CLOCK_HZ
#define GASPETTO_I2C_CLOCK_HZ 100000
#endif

#ifndef GASPETTO_ADC_SAMPLES_PER_CHANNEL
#define GASPETTO_ADC_SAMPLES_PER_CHANNEL 3
#endif

#ifndef GASPETTO_ADC_MIN_VALID_SAMPLES
#define GASPETTO_ADC_MIN_VALID_SAMPLES 2
#endif

#ifndef GASPETTO_ADC_READ_RECOVERY_RETRIES
#define GASPETTO_ADC_READ_RECOVERY_RETRIES 1
#endif

#ifndef GASPETTO_ADC_CONVERSION_TIMEOUT_MS
#define GASPETTO_ADC_CONVERSION_TIMEOUT_MS 40
#endif

/* Piece thresholds come from the shared piece_thresholds.h, with their
 * static asserts. Only sampling and timing knobs stay defined here */

static_assert(GASPETTO_ADC_SAMPLES_PER_CHANNEL > 0,
              "GASPETTO_ADC_SAMPLES_PER_CHANNEL must be >= 1");
static_assert(GASPETTO_ADC_MIN_VALID_SAMPLES > 0, "GASPETTO_ADC_MIN_VALID_SAMPLES must be >= 1");
static_assert(GASPETTO_ADC_MIN_VALID_SAMPLES <= GASPETTO_ADC_SAMPLES_PER_CHANNEL,
              "GASPETTO_ADC_MIN_VALID_SAMPLES must be <= GASPETTO_ADC_SAMPLES_PER_CHANNEL");

constexpr uint8_t kChannelsPerAds = 4;
constexpr uint32_t kI2cClockHz = GASPETTO_I2C_CLOCK_HZ;
constexpr uint8_t kAdcSamplesPerChannel = GASPETTO_ADC_SAMPLES_PER_CHANNEL;
constexpr uint8_t kAdcMinValidSamples = GASPETTO_ADC_MIN_VALID_SAMPLES;
constexpr uint8_t kAdcReadRecoveryRetries = GASPETTO_ADC_READ_RECOVERY_RETRIES;
constexpr uint16_t kAdcConversionTimeoutMs = GASPETTO_ADC_CONVERSION_TIMEOUT_MS;

constexpr std::array<uint16_t, 4> kMuxByChannel = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0,
    ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2,
    ADS1X15_REG_CONFIG_MUX_SINGLE_3,
};

enum class AdsBus : uint8_t {
    BUS_1,
    BUS_3,
};

struct AdsDeviceConfig {
    AdsBus bus;
    uint8_t address;
    uint8_t slotBase;
};

#ifdef ARDUINO
BitBangWire gI2c3(I2C3_SDA_PIN, I2C3_SCL_PIN);
#else
TwoWire gI2c3;
#endif

/* Board slot wiring: slots 1-4 on the 0x4B module (ADDR->SCL), 5-8 on
 * 0x48 (ADDR->GND), 9-12 on 0x4A (ADDR->SDA), 13-16 on 0x49 (ADDR->VCC),
 * 17-20 on the bit-banged I2C3 bus. Mirrors kAdsTests of
 * arduino_box_hw_test and the PC mock slotBaseFor */
constexpr std::array<AdsDeviceConfig, 5> kAdsDevices = {
    AdsDeviceConfig{ AdsBus::BUS_1, 0x4B, 0 },  AdsDeviceConfig{ AdsBus::BUS_1, 0x48, 4 },
    AdsDeviceConfig{ AdsBus::BUS_1, 0x4A, 8 },  AdsDeviceConfig{ AdsBus::BUS_1, 0x49, 12 },
    AdsDeviceConfig{ AdsBus::BUS_3, 0x4A, 16 },
};

/* Expected LED behaviour, three APA106 pixels on the strip, left to right:
 * - boot health check: led 0 (left) lights solid green when every probe passes,
 *   solid red otherwise, holds ~1.5s then goes off
 * - board scan: a cyan dot bounces led 0 -> 1 -> 2 -> back twice, then blackout
 * - program accepted: green cascades led 0 -> 1 -> 2, all three hold green
 * - build failed: led 2 (right) blinks red three times, then holds red
 * - empty board: led 2 blinks amber twice, then holds amber
 * - RF send failed: led 1 (center) blinks red three times, then holds with led
 *   1 red plus led 0 green
 * Terminal states hold their pattern ~60s, then the box arms STOP mode and all
 * LEDs go off (PC emulation runs the same patterns as log-only animations)
 */
constexpr uint8_t kLedState = 0; /* left   : system / scan state   */
constexpr uint8_t kLedRadio = 1; /* center : radio status           */
constexpr uint8_t kLedBuild = 2; /* right  : build / program error  */

TwoWire &wireForAdsBus(AdsBus bus)
{
    return bus == AdsBus::BUS_1 ? Wire : gI2c3;
}

void recoverAdsBus(AdsBus bus)
{
    TwoWire &wire = wireForAdsBus(bus);

    wire.end();
    delay(5);

    if (bus == AdsBus::BUS_1) {
        Wire.setSCL(I2C1_SCL_PIN);
        Wire.setSDA(I2C1_SDA_PIN);
    } else {
        gI2c3.setSCL(I2C3_SCL_PIN);
        gI2c3.setSDA(I2C3_SDA_PIN);
    }

    wire.begin();
    wire.setClock(kI2cClockHz);
    delay(10);
}

bool readSingleEndedWithTimeout(Adafruit_ADS1115 &ads, uint8_t channel, int16_t &raw)
{
    if (channel >= kMuxByChannel.size()) {
        return false;
    }

    ads.startADCReading(kMuxByChannel[channel], false);
    const unsigned long startMs = millis();
    while (!ads.conversionComplete()) {
        if (millis() - startMs > kAdcConversionTimeoutMs) {
            return false;
        }
        delay(1);
    }

    raw = ads.getLastConversionResults();
    return true;
}

const char *busName(AdsBus bus)
{
    return bus == AdsBus::BUS_1 ? "I2C1" : "I2C3";
}
}

GBox::GBox(Context &ctx)
        : GenericActiveObject()
        , _ctx(ctx)
        , lastScan{}
        , initialized_(false)
        , lastDebounceTime_(0)
        , isScanning_(false)
{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GBox::init(StateId initialStateId)
{
    GenericActiveObject::init(initialStateId);
}

int GBox::postEvent(Event evt)
{
    if (_ctx.mainEventQueue) {
        _ctx.mainEventQueue->enqueue(evt);
        return 0;
    }
    return -1;
}

void GBox::work()
{
    /* The box is the receiver of the pair: telemetry, logs and version
     * requests arrive on _box_ and are answered or printed here. Reading one
     * packet per pass keeps the chip acknowledged without delaying the state
     * machine, and it is also what makes the car's log writes cheap */
    if (_ctx.radioController != nullptr) {
        _ctx.radioController->processRadio();
    }

    if (_ctx.mainEventQueue && !_ctx.mainEventQueue->IsEmpty()) {
        Event evt;

        StateType *currentState = states[static_cast<uint8_t>(currentStateIndex)];
        _ctx.mainEventQueue->dequeue(evt);
        currentState->processEvent(evt);
    }
}

void GBox::enterLowPowerMode()
{
    prepareForStop();
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    LOGLN("Entering low-power mode...\n");
    if (lowPowerCallback_) {
        lowPowerCallback_();
    }
#else
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_ResumeTick();
    restoreFromStop();
#endif
#endif
}

void GBox::debounceAndEnqueue(Event &evt, unsigned long currentTime)
{
#ifdef ARDUINO
    if (currentTime - lastDebounceTime_ > debounceDelay) {
        lastDebounceTime_ = currentTime;
        if (!_ctx.mainEventQueue->IsFull()) {
            _ctx.mainEventQueue->enqueue(evt);
            LOGLN("Exiting low-power mode...\n");
        } else {
            LOGLN("Event queue is full! Unable to enqueue event.\n");
        }
    }
#else
    if (!_ctx.mainEventQueue->IsFull()) {
        postEvent(evt);
    } else {
        LOGLN("Event queue is full! Unable to enqueue event.\n");
    }
#endif
}

bool GBox::routeForSlot(std::size_t slot, AdsRouteInfo &route)
{
    for (const AdsDeviceConfig &device : kAdsDevices) {
        const std::size_t slotBase = static_cast<std::size_t>(device.slotBase);
        const std::size_t slotEnd = slotBase + kChannelsPerAds;
        if (slot >= slotBase && slot < slotEnd) {
            route.usesI2c3 = device.bus == AdsBus::BUS_3;
            route.address = device.address;
            route.channel = static_cast<uint8_t>(slot - slotBase);
            return true;
        }
    }

    return false;
}

void GBox::initHardware()
{
    if (initialized_) {
        return;
    }

    configurePins();
    Wire.setSCL(I2C1_SCL_PIN);
    Wire.setSDA(I2C1_SDA_PIN);
    Wire.begin();

    gI2c3.setSCL(I2C3_SCL_PIN);
    gI2c3.setSDA(I2C3_SDA_PIN);
    gI2c3.begin();

    led_strip_begin();

    if (_ctx.radioController != nullptr) {
        _ctx.radioController->init();
    }
    initialized_ = true;
    LOGLN("GBox: hardware initialized.");
}

bool GBox::runHealthCheck()
{
#ifdef ARDUINO
    bool allOk = true;

    /* Power up the ADS1115 modules on the 3V3 rail before probing. */
    setSensorRailEnabled(true);
    delay(100); /* Let the rail settle. */

    LOGLN("--- Hardware health check ---");

    /* Probe I2C1: 4 ADS1115 addresses, slot order 0x4B..0x49. */
    constexpr uint8_t kI2c1Addresses[] = { 0x4B, 0x48, 0x4A, 0x49 };
    for (uint8_t addr : kI2c1Addresses) {
        Wire.beginTransmission(addr);
        bool ack = (Wire.endTransmission() == 0);
        LOG("I2C1 0x");
        LOG(addr, HEX);
        LOG(": ");
        LOGLN(ack ? "[OK]" : "[FAIL]");
        if (!ack)
            allOk = false;
    }

    /* Probe I2C3: 1 ADS1115 via bit-bang. */
    bool i2c3Ok = i2c3_bb_probe(0x4A);
    LOG("I2C3 0x4A: ");
    LOGLN(i2c3Ok ? "[OK]" : "[FAIL]");
    if (!i2c3Ok)
        allOk = false;

    /* Check radio. */
    bool radioOk = false;
    if (_ctx.radioController != nullptr) {
        radioOk = _ctx.radioController->isChipConnected();
    }
    LOG("nRF24: ");
    LOGLN(radioOk ? "[OK]" : "[FAIL]");
    if (!radioOk)
        allOk = false;

    LOGLN("--- Check complete ---");

    showHealthStatus(allOk);
    setSensorRailEnabled(false);
    return allOk;
#else
    return true;
#endif
}

void GBox::showHealthStatus(bool ok)
{
    if (ok) {
        led_strip_set(0, 0, 50, 0); /* Green. */
    } else {
        led_strip_set(0, 50, 0, 0); /* Red. */
    }
    led_strip_show();
    delayMs(1500);
    led_strip_set(0, 0, 0, 0);
    led_strip_show();
}

#ifndef ARDUINO
/* Used for command injection in emulation mode. */
void GBox::injectBoardPieces(const BoxBoardPieces &boardPieces)
{
    for (std::size_t index = 0; index < BOX_TOTAL_SLOTS; ++index) {
        Adafruit_ADS1115::setRawValue(
                index, static_cast<int16_t>(piece_raw_for_slot(static_cast<uint8_t>(index),
                                                               boardPieces[index])));
    }
    LOGLN("GBox: board pieces injected.");
}

/* Used for command injection in emulation mode. */
void GBox::injectRawAdcValues(const std::array<uint16_t, BOX_TOTAL_SLOTS> &rawValues)
{
    for (std::size_t index = 0; index < BOX_TOTAL_SLOTS; ++index) {
        Adafruit_ADS1115::setRawValue(index, static_cast<int16_t>(rawValues[index]));
    }
    LOGLN("GBox: raw ADC values injected.");
}
#endif

void GBox::restoreFromStop()
{
    SystemClock_Config();
    LOGLN("GBox: wake-up sequence complete.");
}

void GBox::SystemClock_Config()
{
#ifdef ARDUINO
    /* STM32 clocks must be restored here after STOP wake-up on target hardware. */
#else
    LOGLN("SystemClock_Config(): emulated clock tree restored after STOP.");
#endif
}

void GBox::configurePins()
{
    pinMode(PIN_MOSFET_5V_LEDS, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);

    pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);

    pinMode(PIN_LED_DATA, OUTPUT);
    digitalWrite(PIN_LED_DATA, LOW);

    pinMode(PIN_WAKE_BUTTON, INPUT_PULLUP);
}

void GBox::setSensorRailEnabled(bool enabled)
{
    digitalWrite(PIN_MOSFET_3V3_SENSORS, enabled ? LOW : HIGH);
    LOG(enabled ? "3.3V sensor rail ON" : "3.3V sensor rail OFF");
    LOGLN();
}

void GBox::setLedRailEnabled(bool enabled)
{
    digitalWrite(PIN_MOSFET_5V_LEDS, enabled ? LOW : HIGH);
    LOG(enabled ? "5V LED rail ON (PB14 pulled low)" : "5V LED rail OFF (PB14 released)");
    LOGLN();
}

void GBox::blackoutLeds()
{
    led_strip_fill(0, 0, 0);
    led_strip_show();
    LOGLN("LEDs: blackout.");
}

void GBox::delayMs(int ms) const
{
    delay(ms);
}

void GBox::runScanAnimation()
{
    setLedRailEnabled(true);
    delayMs(50);

    /* Bouncing cyan on kLedState → kLedRadio → kLedBuild → back, 2 round trips. */
    for (int trip = 0; trip < 2; ++trip) {
        for (int i = 0; i < 3; ++i) {
            led_strip_fill(0, 0, 0);
            led_strip_set(static_cast<uint16_t>(i), 0, 80, 80);
            led_strip_show();
            delayMs(500);
        }
        for (int i = 1; i >= 0; --i) {
            led_strip_fill(0, 0, 0);
            led_strip_set(static_cast<uint16_t>(i), 0, 80, 80);
            led_strip_show();
            delayMs(500);
        }
    }
    blackoutLeds();
}

void GBox::runSuccessAnimation()
{
    setLedRailEnabled(true);
    /* Cascade green: LED0 → LED1 → LED2, then hold all solid green. */
    led_strip_fill(0, 0, 0);
    for (int i = 0; i < 3; ++i) {
        led_strip_set(static_cast<uint16_t>(i), 0, 100, 0);
        led_strip_show();
        delayMs(150);
    }
    /* Long enough to read the colour, short enough that the button still works */
    delayMs(3000);
}

void GBox::runBuildErrorAnimation()
{
    setLedRailEnabled(true);
    /* kLedBuild (right) blinks red ×3, then holds red on kLedBuild. */
    for (int blink = 0; blink < 3; ++blink) {
        led_strip_fill(0, 0, 0);
        led_strip_set(kLedBuild, 150, 0, 0);
        led_strip_show();
        delayMs(90);
        blackoutLeds();
        delayMs(45);
    }
    led_strip_set(kLedBuild, 150, 0, 0);
    led_strip_show();
    /* Long enough to read the colour, short enough that the button still works */
    delayMs(3000);
}

void GBox::runEmptyBoardAnimation()
{
    setLedRailEnabled(true);
    /* kLedBuild (right) blinks amber ×2, then holds amber on kLedBuild. */
    for (int blink = 0; blink < 2; ++blink) {
        led_strip_fill(0, 0, 0);
        led_strip_set(kLedBuild, 100, 60, 0);
        led_strip_show();
        delayMs(200);
        blackoutLeds();
        delayMs(100);
    }
    led_strip_set(kLedBuild, 100, 60, 0);
    led_strip_show();
    /* Long enough to read the colour, short enough that the button still works */
    delayMs(3000);
}

void GBox::runRfErrorAnimation()
{
    setLedRailEnabled(true);
    /* kLedRadio (center) blinks red ×3, then holds: kLedState green + kLedRadio red. */
    for (int blink = 0; blink < 3; ++blink) {
        led_strip_fill(0, 0, 0);
        led_strip_set(kLedRadio, 150, 0, 0);
        led_strip_show();
        delayMs(90);
        blackoutLeds();
        delayMs(45);
    }
    led_strip_set(kLedState, 0, 80, 0);
    led_strip_set(kLedRadio, 150, 0, 0);
    led_strip_show();
    /* Long enough to read the colour, short enough that the button still works */
    delayMs(3000);
}

void GBox::scanSlots()
{
    for (const AdsDeviceConfig &device : kAdsDevices) {
        Adafruit_ADS1115 ads;
        TwoWire &deviceWire = wireForAdsBus(device.bus);

        if (!ads.begin(device.address, &deviceWire)) {
            LOG("ADS init failed [");
            LOG(busName(device.bus));
            LOG(" addr=0x");
            LOG(device.address, HEX);
            LOGLN("]");
            for (uint8_t channel = 0; channel < kChannelsPerAds; ++channel) {
                const std::size_t slot = static_cast<std::size_t>(device.slotBase) + channel;
                lastScan[slot].rawValue = 0;
                lastScan[slot].piece = BoxPieceId::INVALID;
            }
            continue;
        }

        ads.setGain(GAIN_ONE);
        for (uint8_t channel = 0; channel < kChannelsPerAds; ++channel) {
            const std::size_t slot = static_cast<std::size_t>(device.slotBase) + channel;
            bool readOk = false;
            uint16_t rawU16 = 0;
            uint8_t attemptUsed = 0;
            uint8_t bestValidSamples = 0;
            uint8_t timeoutCount = 0;
            uint8_t negativeCount = 0;

            for (uint8_t recoveryAttempt = 0; recoveryAttempt <= kAdcReadRecoveryRetries;
                 ++recoveryAttempt) {
                uint32_t sum = 0;
                uint8_t validSamples = 0;

                for (uint8_t sample = 0; sample < kAdcSamplesPerChannel; ++sample) {
                    int16_t raw = 0;
                    if (!readSingleEndedWithTimeout(ads, channel, raw)) {
                        ++timeoutCount;
                        continue;
                    }

                    if (raw >= 0) {
                        sum += static_cast<uint16_t>(raw);
                        ++validSamples;
                    } else {
                        ++negativeCount;
                    }
                }

                if (validSamples > bestValidSamples) {
                    bestValidSamples = validSamples;
                }

                if (validSamples >= kAdcMinValidSamples) {
                    rawU16 = static_cast<uint16_t>(sum / validSamples);
                    readOk = true;
                    attemptUsed = recoveryAttempt;
                    break;
                }

                if (recoveryAttempt < kAdcReadRecoveryRetries) {
                    LOG("ADC retry [");
                    LOG(busName(device.bus));
                    LOG(" addr=0x");
                    LOG(device.address, HEX);
                    LOG(" ch=");
                    LOG(static_cast<int>(channel));
                    LOG("] valid=");
                    LOG(validSamples);
                    LOG("/");
                    LOG(kAdcSamplesPerChannel);
                    LOG(" timeouts=");
                    LOG(timeoutCount);
                    LOG(" negatives=");
                    LOG(negativeCount);
                    LOGLN();

                    recoverAdsBus(device.bus);
                    if (!ads.begin(device.address, &deviceWire)) {
                        LOG("ADS re-init failed [");
                        LOG(busName(device.bus));
                        LOG(" addr=0x");
                        LOG(device.address, HEX);
                        LOGLN("]");
                        break;
                    }
                    ads.setGain(GAIN_ONE);
                }
            }

            lastScan[slot].rawValue = readOk ? rawU16 : 0;
            lastScan[slot].piece = readOk ? piece_decode(static_cast<uint8_t>(slot),
                                                         static_cast<int16_t>(rawU16)) :
                                            BoxPieceId::INVALID;

            LOG("Scan slot ");
            LOG(static_cast<int>(slot) + 1);
            LOG(" [");
            LOG(busName(device.bus));
            LOG(" addr=0x");
            LOG(device.address, HEX);
            LOG(" ch=");
            LOG(static_cast<int>(channel));
            LOG("]");
            if (readOk) {
                LOG(": ADC(avg)=");
                LOG(lastScan[slot].rawValue);
                LOG(" samples=");
                LOG(bestValidSamples);
                LOG("/");
                LOG(kAdcSamplesPerChannel);
                LOG(" attempt=");
                LOG(attemptUsed + 1);
                LOG(" => ");
                LOGLN(pieceToString(lastScan[slot].piece));
            } else {
                LOG(": ADC read failed valid=");
                LOG(bestValidSamples);
                LOG("/");
                LOG(kAdcSamplesPerChannel);
                LOG(" timeouts=");
                LOG(timeoutCount);
                LOG(" negatives=");
                LOG(negativeCount);
                LOGLN(" => INVALID");
            }
        }
    }
}

bool GBox::buildProgram(CommandPacket &packet, bool &isEmpty)
{
    BoxBoardPieces boardPieces = {};

    for (std::size_t slot = 0; slot < BOX_TOTAL_SLOTS; ++slot) {
        boardPieces[slot] = lastScan[slot].piece;
    }

    const bool built = buildProgramFromPieces(boardPieces, packet, isEmpty);
    if (!built) {
        if (isEmpty) {
            LOGLN("Board scan produced an empty payload.");
        } else {
            LOGLN("Program build failed due to invalid token or payload overflow.");
        }
        return false;
    }

    LOG("Program payload count: ");
    LOG(packet.count);
    LOGLN();
    return true;
}

bool GBox::sendProgram(const CommandPacket &packet)
{
    if (_ctx.radioController == nullptr) {
        LOGLN("Radio controller unavailable.");
        return false;
    }

    LOG("Sending program with ");
    LOG(packet.count);
    LOGLN(" commands.");

    return _ctx.radioController->sendBuffer(&packet, sizeof(packet));
}

void GBox::prepareForStop()
{
    blackoutLeds();
    setLedRailEnabled(false);
    setSensorRailEnabled(false);
    LOGLN("GBox: STOP mode armed.");
}

const char *GBox::pieceToString(BoxPieceId piece) const
{
    switch (piece) {
    case BoxPieceId::EMPTY:
        return "EMPTY";
    case BoxPieceId::FORWARD:
        return "FORWARD";
    case BoxPieceId::BACKWARD:
        return "BACKWARD";
    case BoxPieceId::TURN_RIGHT:
        return "TURN_RIGHT";
    case BoxPieceId::TURN_LEFT:
        return "TURN_LEFT";
    case BoxPieceId::LOOP_CALL:
        return "LOOP_CALL";
    default:
        return "INVALID";
    }
}

bool GBox::sendClearQueueCommand()
{
    if (_ctx.radioController == nullptr) {
        LOGLN("Radio controller unavailable for clear queue command.");
        return false;
    }

    CommandPacket packet{};
    packet.count = 2;
    packet.commands[0] = static_cast<uint8_t>(CommandId::MOTOR_STOP);
    packet.commands[1] = static_cast<uint8_t>(CommandId::QUEUE_CLEAR);

    LOG("Sending MOTOR_STOP and clear queue command.");
    LOGLN();

    return _ctx.radioController->sendBuffer(&packet, sizeof(packet));
}

bool GBox::isCurrentlyScanning() const
{
    return isScanning_;
}

void GBox::startScanning()
{
    isScanning_ = true;
    LOGLN("Scanning started.");
}

void GBox::interruptScanning()
{
    isScanning_ = false;
    LOGLN("Scanning interrupted by button press.");
}

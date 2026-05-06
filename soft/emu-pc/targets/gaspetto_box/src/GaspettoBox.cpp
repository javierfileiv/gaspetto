#include "GaspettoBox.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "CarEvents.h"
#include "CommandPacket.h"
#include "Context.h"
#include "RadioController.h"
#include "pin_definitions.h"

#include <Adafruit_ADS1X15.h>
#include <array>
#include <cstdint>
#ifdef ARDUINO
#include "stm32f4xx_hal.h"

#include <Adafruit_NeoPixel.h>
#endif
#include <Wire.h>

namespace
{
constexpr std::array<AdcDecodeEntry, 7> kAdcDecodeTable = {
    AdcDecodeEntry{ 0, 99, BoxPieceId::EMPTY, "EMPTY" },
    AdcDecodeEntry{ 100, 699, BoxPieceId::FORWARD, "FORWARD" },
    AdcDecodeEntry{ 700, 1299, BoxPieceId::BACKWARD, "BACKWARD" },
    AdcDecodeEntry{ 1300, 1899, BoxPieceId::TURN_RIGHT, "TURN_RIGHT" },
    AdcDecodeEntry{ 1900, 2499, BoxPieceId::TURN_LEFT, "TURN_LEFT" },
    AdcDecodeEntry{ 2500, 3099, BoxPieceId::STOP, "STOP" },
    AdcDecodeEntry{ 3100, 4095, BoxPieceId::LOOP_CALL, "LOOP_CALL" },
};

constexpr uint8_t kChannelsPerAds = 4;
constexpr uint32_t kI2cClockHz = 100000;
constexpr uint8_t kAdcSamplesPerChannel = 3;
constexpr uint8_t kAdcMinValidSamples = 2;
constexpr uint8_t kAdcReadRecoveryRetries = 1;
constexpr uint16_t kAdcConversionTimeoutMs = 40;

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
TwoWire gI2c3(I2C3_SDA_PIN, I2C3_SCL_PIN);
#else
TwoWire gI2c3;
#endif

constexpr std::array<AdsDeviceConfig, 5> kAdsDevices = {
    AdsDeviceConfig{ AdsBus::BUS_1, 0x48, 0 },  AdsDeviceConfig{ AdsBus::BUS_1, 0x49, 4 },
    AdsDeviceConfig{ AdsBus::BUS_1, 0x4A, 8 },  AdsDeviceConfig{ AdsBus::BUS_1, 0x4B, 12 },
    AdsDeviceConfig{ AdsBus::BUS_3, 0x4A, 16 },
};

#ifdef ARDUINO
constexpr uint8_t kLedCount = 3;
constexpr uint8_t kLedState = 0; /* left   : system / scan state   */
constexpr uint8_t kLedRadio = 1; /* center : radio status           */
constexpr uint8_t kLedBuild = 2; /* right  : build / program error  */
#endif

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

CommandId pieceToCommand(BoxPieceId piece)
{
    switch (piece) {
    case BoxPieceId::FORWARD:
        return CommandId::MOTOR_FORWARD;
    case BoxPieceId::BACKWARD:
        return CommandId::MOTOR_BACKWARD;
    case BoxPieceId::TURN_RIGHT:
        return CommandId::MOTOR_TURN_RIGHT;
    case BoxPieceId::TURN_LEFT:
        return CommandId::MOTOR_TURN_LEFT;
    case BoxPieceId::STOP:
        return CommandId::MOTOR_STOP;
    default:
        return CommandId::NONE;
    }
}

bool appendCommand(CommandPacket &packet, CommandId command)
{
    if (packet.count >= BOX_MAX_PAYLOAD_COMMANDS) {
        return false;
    }

    packet.commands[packet.count] = static_cast<uint8_t>(command);
    ++packet.count;
    return true;
}

bool appendPieceCommand(CommandPacket &packet, BoxPieceId piece)
{
    const CommandId command = pieceToCommand(piece);

    if (command == CommandId::NONE) {
        return piece == BoxPieceId::EMPTY;
    }

    return appendCommand(packet, command);
}
}

GaspettoBox::GaspettoBox(Context &ctx)
        : GenericActiveObject()
        , _ctx(ctx)
        , lastScan{}
#ifdef ARDUINO
        , leds_(kLedCount, PIN_LED_DATA, NEO_GRB + NEO_KHZ800)
#endif
        , initialized_(false)
        , lastDebounceTime_(0)
        , isScanning_(false)
{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GaspettoBox::init(StateId initialStateId)
{
    GenericActiveObject::init(initialStateId);
}

int GaspettoBox::postEvent(Event evt)
{
    if (_ctx.mainEventQueue) {
        _ctx.mainEventQueue->enqueue(evt);
        return 0;
    }
    return -1;
}

void GaspettoBox::work()
{
    if (_ctx.mainEventQueue && !_ctx.mainEventQueue->IsEmpty()) {
        Event evt;

        StateType *currentState = states[static_cast<uint8_t>(currentStateIndex)];
        _ctx.mainEventQueue->dequeue(evt);
        currentState->processEvent(evt);
    }
}

void GaspettoBox::enterLowPowerMode()
{
    prepareForStop();
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    logln("Entering low-power mode...\n");
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

void GaspettoBox::debounceAndEnqueue(Event &evt, unsigned long currentTime)
{
#ifdef ARDUINO
    if (currentTime - lastDebounceTime_ > debounceDelay) {
        lastDebounceTime_ = currentTime;
        if (!_ctx.mainEventQueue->IsFull()) {
            _ctx.mainEventQueue->enqueue(evt);
            logln("Exiting low-power mode...\n");
        } else {
            logln("Event queue is full! Unable to enqueue event.\n");
        }
    }
#else
    if (!_ctx.mainEventQueue->IsFull()) {
        postEvent(evt);
    } else {
        logln("Event queue is full! Unable to enqueue event.\n");
    }
#endif
}

bool GaspettoBox::buildProgramFromPieces(const BoxBoardPieces &boardPieces, CommandPacket &packet,
                                         bool &isEmpty)
{
    std::array<BoxPieceId, BOX_LOOP_SLOTS> loopPieces = {};

    packet.count = 0;
    for (std::size_t index = 0; index < BOX_MAX_PAYLOAD_COMMANDS; ++index) {
        packet.commands[index] = static_cast<uint8_t>(CommandId::NONE);
    }

    for (std::size_t slot = 0; slot < BOX_LOOP_SLOTS; ++slot) {
        loopPieces[slot] = boardPieces[BOX_MAIN_SLOTS + slot];
        if (loopPieces[slot] == BoxPieceId::LOOP_CALL || loopPieces[slot] == BoxPieceId::INVALID) {
            isEmpty = false;
            return false;
        }
    }

    for (std::size_t slot = 0; slot < BOX_MAIN_SLOTS; ++slot) {
        const BoxPieceId piece = boardPieces[slot];

        if (piece == BoxPieceId::EMPTY) {
            continue;
        }

        if (piece == BoxPieceId::INVALID) {
            isEmpty = false;
            return false;
        }

        if (piece == BoxPieceId::LOOP_CALL) {
            for (std::size_t loopSlot = 0; loopSlot < BOX_LOOP_SLOTS; ++loopSlot) {
                if (loopPieces[loopSlot] == BoxPieceId::EMPTY) {
                    continue;
                }

                if (!appendPieceCommand(packet, loopPieces[loopSlot])) {
                    isEmpty = false;
                    return false;
                }
            }
            continue;
        }

        if (!appendPieceCommand(packet, piece)) {
            isEmpty = false;
            return false;
        }
    }

    isEmpty = packet.count == 0;
    return !isEmpty;
}

bool GaspettoBox::routeForSlot(std::size_t slot, AdsRouteInfo &route)
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

void GaspettoBox::initHardware()
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

#ifdef ARDUINO
    leds_.begin();
    leds_.clear();
    leds_.show();
#endif

    if (_ctx.radioController != nullptr) {
        _ctx.radioController->init();
    }
    initialized_ = true;
    logln("GaspettoBox: hardware initialized.");
}

#ifndef ARDUINO
/* Used for command injection in emulation mode. */
void GaspettoBox::injectBoardPieces(const BoxBoardPieces &boardPieces)
{
    for (std::size_t index = 0; index < BOX_TOTAL_SLOTS; ++index) {
        Adafruit_ADS1115::setRawValue(index,
                                      static_cast<int16_t>(rawValueForPiece(boardPieces[index])));
    }
    logln("GaspettoBox: board pieces injected.");
}

/* Used for command injection in emulation mode. */
void GaspettoBox::injectRawAdcValues(const std::array<uint16_t, BOX_TOTAL_SLOTS> &rawValues)
{
    for (std::size_t index = 0; index < BOX_TOTAL_SLOTS; ++index) {
        Adafruit_ADS1115::setRawValue(index, static_cast<int16_t>(rawValues[index]));
    }
    logln("GaspettoBox: raw ADC values injected.");
}
#endif

void GaspettoBox::restoreFromStop()
{
    SystemClock_Config();
    logln("GaspettoBox: wake-up sequence complete.");
}

void GaspettoBox::SystemClock_Config()
{
#ifdef ARDUINO
    /* STM32 clocks must be restored here after STOP wake-up on target hardware. */
#else
    logln("SystemClock_Config(): emulated clock tree restored after STOP.");
#endif
}

void GaspettoBox::configurePins()
{
    pinMode(PIN_MOSFET_5V_LEDS, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);

    pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);

    pinMode(PIN_LED_DATA, OUTPUT);
    digitalWrite(PIN_LED_DATA, LOW);

    pinMode(PIN_WAKE_BUTTON, INPUT_PULLUP);
}

void GaspettoBox::setSensorRailEnabled(bool enabled)
{
    digitalWrite(PIN_MOSFET_3V3_SENSORS, enabled ? LOW : HIGH);
    log(enabled ? "3.3V sensor rail ON" : "3.3V sensor rail OFF");
    logln();
}

void GaspettoBox::setLedRailEnabled(bool enabled)
{
    digitalWrite(PIN_MOSFET_5V_LEDS, enabled ? LOW : HIGH);
    log(enabled ? "5V LED rail ON (PB14 pulled low)" : "5V LED rail OFF (PB14 released)");
    logln();
}

void GaspettoBox::blackoutLeds()
{
#ifdef ARDUINO
    leds_.clear();
    leds_.show();
#endif
    logln("LEDs: blackout.");
}

void GaspettoBox::delayMs(int ms) const
{
    delay(ms);
}

void GaspettoBox::runScanAnimation()
{
    setLedRailEnabled(true);
    delayMs(50);

#ifdef ARDUINO
    /* Bouncing cyan on kLedState → kLedRadio → kLedBuild → back, 2 round trips. */
    for (int trip = 0; trip < 2; ++trip) {
        for (int i = 0; i < 3; ++i) {
            leds_.clear();
            leds_.setPixelColor(static_cast<uint16_t>(i), leds_.Color(0, 80, 80));
            leds_.show();
            delayMs(120);
        }
        for (int i = 1; i >= 0; --i) {
            leds_.clear();
            leds_.setPixelColor(static_cast<uint16_t>(i), leds_.Color(0, 80, 80));
            leds_.show();
            delayMs(120);
        }
    }
    blackoutLeds();
#else
    for (std::size_t slot = 0; slot < BOX_LED_SLOTS; ++slot) {
        log("LED scanner CYAN slot ");
        log(static_cast<int>(slot) + 1);
        logln();
        delayMs(15);
    }
#endif
}

void GaspettoBox::runSuccessAnimation()
{
    setLedRailEnabled(true);
#ifdef ARDUINO
    /* Cascade green: LED0 → LED1 → LED2, then hold all solid green ~1min. */
    leds_.clear();
    for (int i = 0; i < 3; ++i) {
        leds_.setPixelColor(static_cast<uint16_t>(i), leds_.Color(0, 100, 0));
        leds_.show();
        delayMs(150);
    }
    delayMs(60000); /* Hold result visible ~1min before STOP. */
#else
    for (int pulse = 0; pulse < 3; ++pulse) {
        log("LED pulse GREEN step ");
        log(pulse + 1);
        logln();
        delayMs(70);
    }
    blackoutLeds();
    setLedRailEnabled(false);
#endif
}

void GaspettoBox::runBuildErrorAnimation()
{
    setLedRailEnabled(true);
#ifdef ARDUINO
    /* kLedBuild (right) blinks red ×3, then holds red on kLedBuild. */
    for (int blink = 0; blink < 3; ++blink) {
        leds_.clear();
        leds_.setPixelColor(kLedBuild, leds_.Color(150, 0, 0));
        leds_.show();
        delayMs(90);
        blackoutLeds();
        delayMs(45);
    }
    leds_.setPixelColor(kLedBuild, leds_.Color(150, 0, 0));
    leds_.show();
    delayMs(60000); /* Hold result visible ~1min before STOP. */
#else
    logln("LED build error: LED2 blinks red.");
    for (int blink = 0; blink < 3; ++blink) {
        log("LED blink RED (build) step ");
        log(blink + 1);
        logln();
        delayMs(90);
    }
    blackoutLeds();
    setLedRailEnabled(false);
#endif
}

void GaspettoBox::runEmptyBoardAnimation()
{
    setLedRailEnabled(true);
#ifdef ARDUINO
    /* kLedBuild (right) blinks amber ×2, then holds amber on kLedBuild. */
    for (int blink = 0; blink < 2; ++blink) {
        leds_.clear();
        leds_.setPixelColor(kLedBuild, leds_.Color(100, 60, 0));
        leds_.show();
        delayMs(200);
        blackoutLeds();
        delayMs(100);
    }
    leds_.setPixelColor(kLedBuild, leds_.Color(100, 60, 0));
    leds_.show();
    delayMs(60000); /* Hold result visible ~1min before STOP. */
#else
    logln("LED empty board: LED2 blinks amber.");
    for (int blink = 0; blink < 2; ++blink) {
        log("LED blink AMBER (empty) step ");
        log(blink + 1);
        logln();
        delayMs(200);
    }
    blackoutLeds();
    setLedRailEnabled(false);
#endif
}

void GaspettoBox::runRfErrorAnimation()
{
    setLedRailEnabled(true);
#ifdef ARDUINO
    /* kLedRadio (center) blinks red ×3, then holds: kLedState green + kLedRadio red. */
    for (int blink = 0; blink < 3; ++blink) {
        leds_.clear();
        leds_.setPixelColor(kLedRadio, leds_.Color(150, 0, 0));
        leds_.show();
        delayMs(90);
        blackoutLeds();
        delayMs(45);
    }
    leds_.setPixelColor(kLedState, leds_.Color(0, 80, 0));
    leds_.setPixelColor(kLedRadio, leds_.Color(150, 0, 0));
    leds_.show();
    delayMs(60000); /* Hold result visible ~1min before STOP. */
#else
    logln("LED radio error: LED1 blinks red.");
    for (int blink = 0; blink < 3; ++blink) {
        log("LED blink RED (radio) step ");
        log(blink + 1);
        logln();
        delayMs(90);
    }
    blackoutLeds();
    setLedRailEnabled(false);
#endif
}

void GaspettoBox::scanSlots()
{
    for (const AdsDeviceConfig &device : kAdsDevices) {
        Adafruit_ADS1115 ads;
        TwoWire &deviceWire = wireForAdsBus(device.bus);

        if (!ads.begin(device.address, &deviceWire)) {
            log("ADS init failed [");
            log(busName(device.bus));
            log(" addr=0x");
            log(device.address, HEX);
            logln("]");
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
                    log("ADC retry [");
                    log(busName(device.bus));
                    log(" addr=0x");
                    log(device.address, HEX);
                    log(" ch=");
                    log(static_cast<int>(channel));
                    log("] valid=");
                    log(validSamples);
                    log("/");
                    log(kAdcSamplesPerChannel);
                    log(" timeouts=");
                    log(timeoutCount);
                    log(" negatives=");
                    log(negativeCount);
                    logln();

                    recoverAdsBus(device.bus);
                    if (!ads.begin(device.address, &deviceWire)) {
                        log("ADS re-init failed [");
                        log(busName(device.bus));
                        log(" addr=0x");
                        log(device.address, HEX);
                        logln("]");
                        break;
                    }
                    ads.setGain(GAIN_ONE);
                }
            }

            lastScan[slot].rawValue = readOk ? rawU16 : 0;
            lastScan[slot].piece = readOk ? decodePiece(rawU16) : BoxPieceId::INVALID;

            log("Scan slot ");
            log(static_cast<int>(slot) + 1);
            log(" [");
            log(busName(device.bus));
            log(" addr=0x");
            log(device.address, HEX);
            log(" ch=");
            log(static_cast<int>(channel));
            log("]");
            if (readOk) {
                log(": ADC(avg)=");
                log(lastScan[slot].rawValue);
                log(" samples=");
                log(bestValidSamples);
                log("/");
                log(kAdcSamplesPerChannel);
                log(" attempt=");
                log(attemptUsed + 1);
                log(" => ");
                logln(pieceToString(lastScan[slot].piece));
            } else {
                log(": ADC read failed valid=");
                log(bestValidSamples);
                log("/");
                log(kAdcSamplesPerChannel);
                log(" timeouts=");
                log(timeoutCount);
                log(" negatives=");
                log(negativeCount);
                logln(" => INVALID");
            }
        }
    }
}

bool GaspettoBox::buildProgram(CommandPacket &packet, bool &isEmpty)
{
    BoxBoardPieces boardPieces = {};

    for (std::size_t slot = 0; slot < BOX_TOTAL_SLOTS; ++slot) {
        boardPieces[slot] = lastScan[slot].piece;
    }

    const bool built = buildProgramFromPieces(boardPieces, packet, isEmpty);
    if (!built) {
        if (isEmpty) {
            logln("Board scan produced an empty payload.");
        } else {
            logln("Program build failed due to invalid token or payload overflow.");
        }
        return false;
    }

    log("Program payload count: ");
    log(packet.count);
    logln();
    return true;
}

bool GaspettoBox::sendProgram(const CommandPacket &packet)
{
    if (_ctx.radioController == nullptr) {
        logln("Radio controller unavailable.");
        return false;
    }

    log("Sending program with ");
    log(packet.count);
    logln(" commands.");

    return _ctx.radioController->sendBuffer(&packet, sizeof(packet));
}

void GaspettoBox::prepareForStop()
{
    blackoutLeds();
    setLedRailEnabled(false);
    setSensorRailEnabled(false);
    logln("GaspettoBox: STOP mode armed.");
}

uint16_t GaspettoBox::rawValueForPiece(BoxPieceId piece) const
{
    for (const AdcDecodeEntry &entry : kAdcDecodeTable) {
        if (entry.piece == piece) {
            return static_cast<uint16_t>((entry.minValue + entry.maxValue) / 2U);
        }
    }

    return 0;
}

BoxPieceId GaspettoBox::decodePiece(uint16_t rawValue) const
{
    for (const AdcDecodeEntry &entry : kAdcDecodeTable) {
        if (rawValue >= entry.minValue && rawValue <= entry.maxValue) {
            return entry.piece;
        }
    }

    return BoxPieceId::INVALID;
}

const char *GaspettoBox::pieceToString(BoxPieceId piece) const
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
    case BoxPieceId::STOP:
        return "STOP";
    case BoxPieceId::LOOP_CALL:
        return "LOOP_CALL";
    default:
        return "INVALID";
    }
}

bool GaspettoBox::sendClearQueueCommand()
{
    if (_ctx.radioController == nullptr) {
        logln("Radio controller unavailable for clear queue command.");
        return false;
    }

    CommandPacket packet{};
    packet.count = 2;
    packet.commands[0] = static_cast<uint8_t>(CommandId::QUEUE_CLEAR);
    packet.commands[1] = static_cast<uint8_t>(CommandId::MOTOR_STOP);

    log("Sending clear queue command with MOTOR_STOP.");
    logln();

    return _ctx.radioController->sendBuffer(&packet, sizeof(packet));
}

bool GaspettoBox::isCurrentlyScanning() const
{
    return isScanning_;
}

void GaspettoBox::startScanning()
{
    isScanning_ = true;
    logln("Scanning started.");
}

void GaspettoBox::interruptScanning()
{
    isScanning_ = false;
    logln("Scanning interrupted by button press.");
}

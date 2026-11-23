#include "GBox.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "CommandPacket.h"
#include "Context.h"
#include "Events.h"
#include "PieceProgram.h"
#include "RadioController.h"
#include "RadioProtocol.h"
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

/* The idle panel is repainted at this cadence, and the lost link blinks with
 * this half-period: slow enough to read, fast enough to notice */
constexpr uint32_t kStatusPaintMs = 100;
constexpr uint32_t kLinkBlinkMs = 400;
/* The program's pixel blinks at this half-period while the car runs */
constexpr uint32_t kProgramBlinkMs = 500;
/* The car's end notice can be lost: the blink must not run forever. Fifteen
 * seconds covers a short program; a longer one needs this raised or scaled
 * with the command count */
constexpr uint32_t kProgramDoneTimeoutMs = 15000;

/* Expected LED behaviour, three APA106 pixels on the strip, bottom to top:
 * - boot health check: the bottom pixel lights solid green when every probe
 *   passes, solid red otherwise, holds ~1.5s then goes off
 * - board scan: a cyan dot climbs bottom -> middle -> top and back twice, then
 *   blackout
 * - program accepted: a green flash on the middle pixel, the link's role
 * - build failed: the bottom pixel blinks red three times, then holds red
 * - empty board: the bottom pixel blinks amber twice, then holds amber
 * - RF send failed: the middle pixel blinks red three times, then holds red
 *   with the bottom pixel green
 * Terminal states hold their pattern ~3s, then the box goes back to idle and
 * the rail keeps the button LED lit
 *
 * The strip is mounted vertically and read as the signal path: the box at the
 * bottom, the radio in the middle, the car at the top. The chain's data input
 * is at the top on this board, so index 0 is the top pixel and index 2 the
 * bottom one. If the strip is wired the other way round, swapping these three
 * constants is enough for every animation to follow.
 */
constexpr uint8_t kLedCar = 0; /* top    : the car and its run   */
constexpr uint8_t kLedLink = 1; /* middle : the radio link        */
constexpr uint8_t kLedBuild = 2; /* bottom : the board and build   */

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
        , programRunning_(false)
        , programAnimMs_(0)
        , programStartedMs_(0)
        , programAnimStep_(0)
        , systemOk_(true)
        , lastVersionSeenMs_(0)
        , statusPaintedMs_(0)
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

    /* The breath and the idle panel run from the loop, so the box stays
     * responsive while the car drives the program */
    serviceProgramAnimation();
    serviceLink();
    serviceStatus();

    if (_ctx.mainEventQueue && !_ctx.mainEventQueue->IsEmpty()) {
        Event evt;

        StateType *currentState = states[static_cast<uint8_t>(currentStateIndex)];
        _ctx.mainEventQueue->dequeue(evt);
        currentState->processEvent(evt);
    }
}

void GBox::enterLowPowerMode()
{
#ifdef LOW_POWER_MODE
    prepareForStop();
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
#else
    /* Stop is not implemented yet, so the 5V rail stays up and the button's
     * LED stays lit; only the pixel chain is cleared. The chaser keeps its
     * own rail while the car runs the program */
    if (!programRunning_) {
        blackoutLeds();
    }
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

    systemOk_ = allOk;
    showHealthStatus(allOk);
    setSensorRailEnabled(false);
    return allOk;
#else
    systemOk_ = true;
    return true;
#endif
}

void GBox::showHealthStatus(bool ok)
{
    if (ok) {
        led_strip_set(kLedBuild, 0, 50, 0); /* Green. */
    } else {
        led_strip_set(kLedBuild, 50, 0, 0); /* Red. */
    }
    led_strip_show();
    delayMs(1500);
    led_strip_set(kLedBuild, 0, 0, 0);
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

    /* Bouncing cyan from bottom to top and back, 2 round trips: the scan
     * belongs to the box, and the motion reads along the column */
    for (int trip = 0; trip < 2; ++trip) {
        for (int i = 0; i < 3; ++i) {
            led_strip_fill(0, 0, 0);
            led_strip_set(static_cast<uint16_t>(i), 0, 80, 80);
            led_strip_show();
            delayMs(120);
        }
        for (int i = 1; i >= 0; --i) {
            led_strip_fill(0, 0, 0);
            led_strip_set(static_cast<uint16_t>(i), 0, 80, 80);
            led_strip_show();
            delayMs(120);
        }
    }
    blackoutLeds();
}

void GBox::runSuccessAnimation()
{
    setLedRailEnabled(true);
    /* An acknowledged send says the link works, which is the middle pixel's
     * role: the other two keep their own meaning. A short flash is enough,
     * and the yellow breath follows at once while the car drives */
    led_strip_fill(0, 0, 0);
    led_strip_set(kLedLink, 0, 100, 0);
    led_strip_show();
    delayMs(150);
}

void GBox::runBuildErrorAnimation()
{
    setLedRailEnabled(true);
    /* The bottom pixel blinks red ×3, then holds: building the program is the
     * box's job, so the failure sits on its own pixel */
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
    /* The bottom pixel blinks amber ×2, then holds: there was nothing on the
     * board to build from */
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
    /* The middle pixel blinks red ×3, then holds red with the bottom pixel
     * green: the board is fine, the link is not */
    for (int blink = 0; blink < 3; ++blink) {
        led_strip_fill(0, 0, 0);
        led_strip_set(kLedLink, 150, 0, 0);
        led_strip_show();
        delayMs(90);
        blackoutLeds();
        delayMs(45);
    }
    led_strip_set(kLedBuild, 0, 80, 0);
    led_strip_set(kLedLink, 150, 0, 0);
    led_strip_show();
    /* Long enough to read the colour, short enough that the button still works */
    delayMs(3000);
}

void GBox::runProgramDoneAnimation()
{
    programRunning_ = false;
    setLedRailEnabled(true);
    delayMs(50);

    /* Two green pulses, then hold: the car finished the batch it was given.
     * Unlike the acknowledgement, this is the real end of the program */
    for (int pulse = 0; pulse < 2; ++pulse) {
        led_strip_fill(0, 0, 0);
        for (uint16_t i = 0; i < LED_STRIP_COUNT; ++i) {
            led_strip_set(i, 0, 100, 0);
        }
        led_strip_show();
        delayMs(300);
        blackoutLeds();
        delayMs(150);
    }
    for (uint16_t i = 0; i < LED_STRIP_COUNT; ++i) {
        led_strip_set(i, 0, 100, 0);
    }
    led_strip_show();
    delayMs(1500);
    blackoutLeds();
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

            /* A program already running keeps its breath: the slot reads block
             * the loop, and freezing the strip for the whole scan would look
             * like the car stopped */
            serviceProgramAnimation();
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

    const bool wasRunning = programRunning_;
    const bool sent = _ctx.radioController->sendBuffer(&packet, sizeof(packet));

    /* The car appends a program sent while another runs, so the breath must not
     * jump back to black for it. A failed send does not cancel the run either:
     * the car is still driving the previous program */
    if (sent) {
        /* An appended program extends the run, so the cap restarts with every
         * accepted send instead of expiring mid-program */
        programStartedMs_ = millis();

        if (!wasRunning) {
            programAnimMs_ = millis();
            programAnimStep_ = 0;
        }
    }

    programRunning_ = sent || wasRunning;
    return sent;
}

void GBox::serviceProgramAnimation()
{
    if (!programRunning_) {
        return;
    }

    const uint32_t now = millis();

    /* The end notice never came: stop pretending the car is still working */
    if ((now - programStartedMs_) >= kProgramDoneTimeoutMs) {
        programRunning_ = false;
        led_strip_set(kLedCar, 0, 0, 0);
        led_strip_show();
        LOGLN("ProgramDone timeout, the yellow blink stops here.");
        return;
    }

    if ((now - programAnimMs_) < kProgramBlinkMs) {
        return;
    }
    programAnimMs_ = now;

    /* The car's pixel is the one that speaks for the car: it blinks yellow
     * while the program runs, and the box and link pixels keep saying their
     * own state instead of being borrowed by the animation */
    const bool lit = (programAnimStep_ == 0);

    led_strip_set(kLedCar, lit ? 180 : 0, lit ? 130 : 0, 0);
    led_strip_show();
    programAnimStep_ = static_cast<uint8_t>((programAnimStep_ + 1U) % 2U);
}

void GBox::serviceLink()
{
    if (_ctx.radioController == nullptr) {
        return;
    }

    const uint32_t now = millis();

    /* RadioController stamps every version answer, including the manual
     * `car` one: a new stamp is the proof the car is still there, and the
     * monitor only needs the arrival time, not the request that caused it */
    const uint32_t answerMs = _ctx.radioController->lastVersionMs();
    if (answerMs != 0 && answerMs != lastVersionSeenMs_) {
        lastVersionSeenMs_ = answerMs;
        linkMonitor_.noteAnswer(answerMs);
    }

    if (!linkMonitor_.service(now)) {
        return;
    }

    VersionPacket request = {};

    request.tag = static_cast<uint8_t>(PacketTag::VersionRequest);
    _ctx.radioController->sendBuffer(&request, sizeof(request));
}

void GBox::serviceStatus()
{
    const uint32_t now = millis();
    if ((now - statusPaintedMs_) < kStatusPaintMs) {
        return;
    }
    statusPaintedMs_ = now;

    const bool linkAlive = linkMonitor_.state() == LinkMonitor::State::Alive;

    /* Bottom pixel: the board itself, green after a passing health check */
    if (systemOk_) {
        led_strip_set(kLedBuild, 0, 60, 0);
    } else {
        led_strip_set(kLedBuild, 140, 0, 0);
    }

    /* Middle pixel: the link, steady green while fresh, blinking red once the
     * car has been silent for too long */
    if (linkAlive) {
        led_strip_set(kLedLink, 0, 60, 0);
    } else if (((now / kLinkBlinkMs) % 2U) == 0U) {
        led_strip_set(kLedLink, 140, 0, 0);
    }

    /* Top pixel: the program animation owns it while the car works, so only
     * clear it once the run is over */
    if (!programRunning_) {
        led_strip_set(kLedCar, 0, 0, 0);
    }

    led_strip_show();
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

#include "Arduino.h"
#include "EventQueue.h"
#include "Events.h"
#include "FirmwareVersion.h"
#include "GBox.h"
#include "IdleState.h"
#include "Log.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "TimeredEventQueue.h"
#include "WireVersion.h"
#include "config_radio.h"
#include "dfu_bootloader.h"
#include "pin_definitions.h"

#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstring>

#ifdef ARDUINO
#include "stm32f4xx_hal.h"
#endif

#ifndef ARDUINO
/* External functions defined in arduino_framework/main.cpp. */
extern Event getEmulatedEvent(void);
#endif

RF24 radio(NRF24_CE, NRF24_CSN);

/* Counted in the button interrupt and printed by the loop: Serial is far too
 * slow and blocking to be called from an ISR, but the count still says
 * whether the pin produced an edge nobody asked for. The counters exist on
 * the host build too, they simply stay at zero there */
volatile uint32_t button_edges = 0;
volatile uint32_t button_edge_ms = 0;
volatile uint8_t button_edge_level = 0;
volatile bool button_press_seen = false;
IdleState idleState;
ProcessingState processingState;
TimeredEventQueue timeredEventQueue;
EventQueue eventQueue;
/* Pipe names designate the destination: the Box transmits commands on the
 * Car's pipe (_car_) and listens on its own (_box_). Passing them in the
 * GCar order made the Box listen where the Car listens and transmit where
 * nobody listens. */
RadioController radioController(radio, &eventQueue, gcar_pipe_name, gbox_pipe_name);
Context context = {
    &eventQueue, &timeredEventQueue, &radioController, &idleState, &processingState,
};
GBox gbox(context);

#ifdef ARDUINO
namespace
{
constexpr std::size_t kUsbCommandBufferSize = 32;

bool isUsbWhitespace(char value)
{
    return value == ' ' || value == '\t' || value == '\r' || value == '\n';
}

bool commandEqualsIgnoreCase(const char *lhs, const char *rhs)
{
    while (*lhs != '\0' && *rhs != '\0') {
        if (std::tolower(static_cast<unsigned char>(*lhs)) !=
            std::tolower(static_cast<unsigned char>(*rhs))) {
            return false;
        }
        ++lhs;
        ++rhs;
    }

    return *lhs == '\0' && *rhs == '\0';
}

void printFirmwareVersion()
{
    LOG(F("GBox firmware build "));
    LOG(FW_VERSION);
    LOG(F(" ("));
    LOG(FW_BUILD);
    LOG(F("), wire protocol v"));
    LOGLN(kWireProtocolVersion);
}

void handleUsbMaintenanceCommands()
{
    static char commandBuffer[kUsbCommandBufferSize] = {};
    static std::size_t commandLength = 0;
    static bool lastWasCarriageReturn = false;

    while (Serial.available() > 0) {
        const int rawValue = Serial.read();
        if (rawValue < 0) {
            break;
        }

        const char currentChar = static_cast<char>(rawValue);
        if (currentChar == '\r' || currentChar == '\n') {
            /* Echo the line end once: a terminal sends CR, LF or both */
            if (!(currentChar == '\n' && lastWasCarriageReturn)) {
                Serial.println();
            }
            lastWasCarriageReturn = (currentChar == '\r');

            commandBuffer[commandLength] = '\0';

            std::size_t start = 0;
            while (start < commandLength && isUsbWhitespace(commandBuffer[start])) {
                ++start;
            }

            std::size_t end = commandLength;
            while (end > start && isUsbWhitespace(commandBuffer[end - 1])) {
                --end;
            }
            commandBuffer[end] = '\0';

            if (end > start) {
                const char *command = &commandBuffer[start];
                if (commandEqualsIgnoreCase(command, "dfu") ||
                    commandEqualsIgnoreCase(command, "reboot-dfu")) {
                    LOGLN("Entering STM32 DFU bootloader...");
                    enterDfuBootloader();
                } else if (commandEqualsIgnoreCase(command, "ver") ||
                           commandEqualsIgnoreCase(command, "version")) {
                    printFirmwareVersion();
                } else if (commandEqualsIgnoreCase(command, "car")) {
                    /* Same version request the NrfLink sends: the car answers
                     * on _box_ and RadioController prints it when it arrives */
                    VersionPacket request = {};

                    request.tag = static_cast<uint8_t>(PacketTag::VersionRequest);
                    radioController.requestVersionLog();
                    const bool sent = radioController.sendBuffer(&request, sizeof(request));

                    LOGLN(sent ? "car version request ACKed, the answer follows" :
                                 "car version request FAILED (no ACK)");
                } else if (commandEqualsIgnoreCase(command, "idle")) {
                    /* Toggle, because the shell must stay reachable while the
                     * radio is off: a second idle brings the link back */
                    const bool quiet = !radioController.isQuiet();

                    radioController.setQuiet(quiet);
                    LOGLN(quiet ? "Radio idle: nothing is sent or received, type idle again to resume" :
                                  "Radio idle: off, listening again");
                } else if (commandEqualsIgnoreCase(command, "reset")) {
                    /* A plain reset, not the bootloader: the line has to leave
                     * before the core goes, or the reason for the reset is the
                     * only thing missing from the console */
                    LOGLN("Resetting the MCU...");
                    Serial.flush();
                    delay(10);
                    NVIC_SystemReset();
                } else if (commandEqualsIgnoreCase(command, "help") ||
                           commandEqualsIgnoreCase(command, "?")) {
                    LOGLN("USB commands: dfu, reboot-dfu, reset, help, ver, car, idle, ?");
                } else {
                    LOG("Unknown USB command: ");
                    LOGLN(command);
                }
            }

            commandLength = 0;
            commandBuffer[0] = '\0';
            continue;
        }
        lastWasCarriageReturn = false;

        if (currentChar == '\b' || currentChar == 0x7F) {
            if (commandLength > 0) {
                --commandLength;
                Serial.print("\b \b");
            }
            continue;
        }

        if (commandLength + 1 >= kUsbCommandBufferSize) {
            commandLength = 0;
            commandBuffer[0] = '\0';
            LOGLN("USB command too long; buffer cleared.");
            continue;
        }

        /* Echo what is typed: the port has no local echo, and without it the
         * operator cannot tell a dead keyboard from a dead command */
        if (currentChar >= ' ' && currentChar <= '~') {
            Serial.print(currentChar);
        }
        commandBuffer[commandLength++] = currentChar;
    }
}
} // namespace
#else
void handleUsbMaintenanceCommands()
{
}
#endif

/* The car reports the end of a command batch on the radio; the box animates
 * it here, where the state machine and the LEDs live */
void onProgramDone()
{
    gbox.runProgramDoneAnimation();
}

/* Arduino-specific function for button interrupt */
void ISR()
{
#ifdef ARDUINO
    /* Record the edge and the level it left behind, then let the loop print
     * it: a press holds the line low, a rail transient is gone by the time
     * the interrupt runs */
    ++button_edges;
    button_edge_ms = millis();
    button_edge_level = (digitalRead(PIN_WAKE_BUTTON) == HIGH) ? 1 : 0;
    /* A real press holds the line low; the spike a rail switch couples into
     * it is gone by the time the interrupt runs, so it reads high here */
    if (button_edge_level == 0) {
        button_press_seen = true;
    }
#else
    Event evt = getEmulatedEvent();
    gbox.debounceAndEnqueue(evt, millis());
#endif
}

void enter_low_power_mode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    LOGLN("Entering low-power mode...\n");
    SwitchToLowPowerMode();
#else
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_ResumeTick();
#endif
#endif
}

void setup()
{
    Serial.begin(115200);
#ifdef GASPETTO_LOG_OVER_NRF24
    Log::attachNrf24(radio, gcar_pipe_name, gcar_pipe_name);
#endif
#ifdef ARDUINO
    /* Wait for CDC USB enumeration on Arduino before printing. */
    const unsigned long deadline = millis() + 1500;
    while (!Serial && millis() < deadline) {
        delay(10);
    }
    LOGLN("GBox BlackPill boot");
    LOGLN("USB commands: dfu, reboot-dfu, reset, help, ver, car, idle, ?");
#else
    LOGLN("Gaspetto Box Initialized");
    LOGLN("Starting up...");
    LOGLN("Commands: P wake, F fail next RF TX\n");
#endif /* ARDUINO */

    gbox.initHardware();
    gbox.runHealthCheck();
    radioController.setProgramDoneHandler(onProgramDone);
    radioController.setLogPeerPrefix("[CAR] ");

    printFirmwareVersion();
    /* Initialize the GBox state machine. */
#ifndef ARDUINO
    gbox.setLowPowerModeCallback(enter_low_power_mode);
#endif
    gbox.init(StateId::IDLE);

    /* Set up ISR for wake button. */
    attachInterrupt(digitalPinToInterrupt(PIN_WAKE_BUTTON), ISR, FALLING);
}

void loop()
{
    static uint32_t printed_edges = 0;
    const uint32_t edges = button_edges;

    if (edges != printed_edges) {
        printed_edges = edges;
        LOG(F("button edge #"));
        LOG(edges);
        LOG(F(" at "));
        LOG(static_cast<uint32_t>(button_edge_ms));
        LOG(F(" ms, pin "));
        LOGLN(button_edge_level != 0 ? F("HIGH") : F("LOW"));
    }

    /* Queue from the loop, not from the interrupt, but keep the press seen
     * by the ISR: a real press during a scan or a verdict animation is then
     * still accepted once the loop comes back */
    if (button_press_seen) {
        button_press_seen = false;
        Event evt(EventId::BUTTON_PRESSED);
        gbox.debounceAndEnqueue(evt, millis());
    }
    handleUsbMaintenanceCommands();
    gbox.work();
}

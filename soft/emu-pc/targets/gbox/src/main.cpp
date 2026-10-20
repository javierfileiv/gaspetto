#include "Arduino.h"
#include "EventQueue.h"
#include "Events.h"
#include "GBox.h"
#include "IdleState.h"
#include "Log.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "TimeredEventQueue.h"
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
IdleState idleState;
ProcessingState processingState;
TimeredEventQueue timeredEventQueue;
EventQueue eventQueue;
RadioController radioController(radio, &eventQueue, gbox_pipe_name, gcar_pipe_name);
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

void handleUsbMaintenanceCommands()
{
    static char commandBuffer[kUsbCommandBufferSize] = {};
    static std::size_t commandLength = 0;

    while (Serial.available() > 0) {
        const int rawValue = Serial.read();
        if (rawValue < 0) {
            break;
        }

        const char currentChar = static_cast<char>(rawValue);
        if (currentChar == '\r' || currentChar == '\n') {
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
                } else if (commandEqualsIgnoreCase(command, "help")) {
                    LOGLN("USB commands: dfu, reboot-dfu, help");
                } else {
                    LOG("Unknown USB command: ");
                    LOGLN(command);
                }
            }

            commandLength = 0;
            commandBuffer[0] = '\0';
            continue;
        }

        if (commandLength + 1 >= kUsbCommandBufferSize) {
            commandLength = 0;
            commandBuffer[0] = '\0';
            LOGLN("USB command too long; buffer cleared.");
            continue;
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

#ifdef ARDUINO
/* Arduino-specific function for button interrupt */
void wakeButton()
{
    Event evt(EventId::BUTTON_PRESSED);
    gbox.debounceAndEnqueue(evt, millis());
}
#else
/* Button press simulation thread. */
void ISR(void)
{
    Event evt = getEmulatedEvent();
    gbox.debounceAndEnqueue(evt, millis());
}
#endif /* ARDUINO */

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
    Log::attachNrf24(radio, gbox_log_pipe_name, gbox_pipe_name);
#endif
#ifdef ARDUINO
    /* Wait for CDC USB enumeration on Arduino before printing. */
    const unsigned long deadline = millis() + 1500;
    while (!Serial && millis() < deadline) {
        delay(10);
    }
    LOGLN("GBox BlackPill boot");
    LOGLN("USB commands: dfu, reboot-dfu, help");
#else
    LOGLN("Gaspetto Box Initialized");
    LOGLN("Starting up...");
    LOGLN("Commands: P wake, F fail next RF TX\n");
#endif /* ARDUINO */

    gbox.initHardware();
    gbox.runHealthCheck();
    /* Initialize the GBox state machine. */
#ifndef ARDUINO
    gbox.setLowPowerModeCallback(enter_low_power_mode);
#endif
    gbox.init(StateId::IDLE);

#ifdef ARDUINO
    /* Set up ISR for wake button. */
    attachInterrupt(digitalPinToInterrupt(PIN_WAKE_BUTTON), wakeButton, FALLING);
#else
    /* Set up ISR for button press simulation. */
    attachInterrupt(digitalPinToInterrupt(PIN_WAKE_BUTTON), ISR, FALLING);
#endif /* ARDUINO */
}

void loop()
{
    handleUsbMaintenanceCommands();
    gbox.work();
}

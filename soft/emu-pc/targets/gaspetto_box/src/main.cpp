#include "Arduino.h"
#include "CarEvents.h"
#include "EventQueue.h"
#include "GaspettoBox.h"
#include "IdleState.h"
#include "Log.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "TimeredEventQueue.h"
#include "config_radio.h"
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
RadioController radioController(radio, &eventQueue, gaspetto_box_pipe_name, gaspetto_car_pipe_name);
Context context = {
    &eventQueue, &timeredEventQueue, &radioController, &idleState, &processingState,
};
GaspettoBox gaspetto_box(context);
Log mainLog;

#ifdef ARDUINO
namespace
{
constexpr uint32_t kDfuRequestMagic = 0x44554631UL;
constexpr uint32_t kSystemMemoryBase = 0x1FFF0000UL;
constexpr std::size_t kUsbCommandBufferSize = 32;

struct DfuResetState {
    uint32_t magic;
};

__attribute__((section(".noinit"))) DfuResetState gDfuResetState;

using BootloaderEntryPoint = void (*)(void);

void clearDfuResetRequest()
{
    gDfuResetState.magic = 0;
}

bool hasSoftwareResetFlag()
{
    return __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET;
}

void requestDfuOnNextBoot()
{
    gDfuResetState.magic = kDfuRequestMagic;
}

void resetIntoDfuBootloader()
{
    requestDfuOnNextBoot();
    Serial.flush();
    delay(20);
    NVIC_SystemReset();
}

[[noreturn]] void jumpToSystemBootloader()
{
    const uint32_t systemMemoryStack = *reinterpret_cast<uint32_t *>(kSystemMemoryBase);
    const uint32_t systemMemoryResetHandler = *reinterpret_cast<uint32_t *>(kSystemMemoryBase + 4U);
    const BootloaderEntryPoint bootloaderEntry =
            reinterpret_cast<BootloaderEntryPoint>(systemMemoryResetHandler);

    Serial.flush();
    Serial.end();
    delay(20);

    __disable_irq();

    for (uint32_t irq = 0; irq < 8; ++irq) {
        NVIC->ICER[irq] = 0xFFFFFFFFUL;
        NVIC->ICPR[irq] = 0xFFFFFFFFUL;
    }

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    HAL_RCC_DeInit();
    HAL_DeInit();

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    SCB->VTOR = kSystemMemoryBase;
    __set_MSP(systemMemoryStack);
    __DSB();
    __ISB();

    bootloaderEntry();

    while (true) {
    }
}

void checkForPendingDfuBoot()
{
    const bool shouldEnterDfu = gDfuResetState.magic == kDfuRequestMagic && hasSoftwareResetFlag();
    clearDfuResetRequest();
    __HAL_RCC_CLEAR_RESET_FLAGS();

    if (shouldEnterDfu) {
        jumpToSystemBootloader();
    }
}

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
                    mainLog.logln("Entering STM32 DFU bootloader...");
                    resetIntoDfuBootloader();
                } else if (commandEqualsIgnoreCase(command, "help")) {
                    mainLog.logln("USB commands: dfu, reboot-dfu, help");
                } else {
                    mainLog.log("Unknown USB command: ");
                    mainLog.logln(command);
                }
            }

            commandLength = 0;
            commandBuffer[0] = '\0';
            continue;
        }

        if (commandLength + 1 >= kUsbCommandBufferSize) {
            commandLength = 0;
            commandBuffer[0] = '\0';
            mainLog.logln("USB command too long; buffer cleared.");
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
    gaspetto_box.debounceAndEnqueue(evt, millis());
}
#else
/* Button press simulation thread. */
void ISR(void)
{
    Event evt = getEmulatedEvent();
    gaspetto_box.debounceAndEnqueue(evt, millis());
}
#endif /* ARDUINO */

void enter_low_power_mode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    mainLog.logln("Entering low-power mode...\n");
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
#ifdef ARDUINO
    checkForPendingDfuBoot();
#endif

    Serial.begin(115200);
#ifdef GASPETTO_LOG_OVER_NRF24
    Log::attachNrf24(radio, gaspetto_box_log_pipe_name, gaspetto_box_pipe_name);
#endif
#ifdef ARDUINO
    /* Wait for CDC USB enumeration on Arduino before printing. */
    const unsigned long deadline = millis() + 1500;
    while (!Serial && millis() < deadline) {
        delay(10);
    }
    mainLog.logln("GaspettoBox BlackPill boot");
    mainLog.logln("USB commands: dfu, reboot-dfu, help");
#else
    mainLog.logln("Gaspetto Box Initialized");
    mainLog.logln("Starting up...");
    mainLog.logln("Commands: P wake, F fail next RF TX\n");
#endif /* ARDUINO */

    gaspetto_box.initHardware();
    /* Initialize the GaspettoBox state machine. */
#ifndef ARDUINO
    gaspetto_box.setLowPowerModeCallback(enter_low_power_mode);
#endif
    gaspetto_box.init(StateId::IDLE);

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
    gaspetto_box.work();
}

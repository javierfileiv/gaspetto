#include "Arduino.h"
#include "CarEvents.h"
#include "EventQueue.h"
#include "GaspettoBox.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "TimeredEventQueue.h"
#include "config_radio.h"
#include "pin_definitions.h"

#include <atomic>

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
    Serial.println("Entering low-power mode...\n");
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
#ifdef ARDUINO
    /* Wait for CDC USB enumeration on Arduino before printing. */
    const unsigned long deadline = millis() + 1500;
    while (!Serial && millis() < deadline) {
        delay(10);
    }
    Serial.println("GaspettoBox BlackPill boot");
#else
    Serial.println("Gaspetto Box Initialized");
    Serial.println("Starting up...");
    Serial.println("Commands: P wake, F fail next RF TX\n");
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
    gaspetto_box.work();
}

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

#include <atomic>

/* External functions defined in arduino_framework/main.cpp. */
extern Event getEmulatedEvent(void);

RF24 radio(CE_PIN, CSN_PIN);
IdleState idleState;
ProcessingState processingState;
TimeredEventQueue timeredEventQueue;
EventQueue eventQueue;
RadioController radioController(radio, &eventQueue, gaspetto_box_pipe_name, gaspetto_car_pipe_name);
Context context = {
    &eventQueue, &timeredEventQueue, &radioController, &idleState, &processingState,
};
GaspettoBox gaspetto_box(context);

/*  Button press simulation thread. */
void ISR(void)
{
#ifndef ARDUINO
    Event evt = getEmulatedEvent();
    gaspetto_box.debounceAndEnqueue(evt, millis());
#endif
}

void enter_low_power_mode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    Serial.println("Entering low-power mode...\n");
    SwitchToLowPowerMode();
#else
    /*  Implement low-power mode for Arduino. */
    /*  STM32 sleep modes or power-saving. */
    delay(100); /*  Simulate low-power sleep. */
#endif
#endif
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Gaspetto Box Initialized");
    Serial.println("Starting up...\n");
    /* Initialize the GaspettoBox state machine. */
    gaspetto_box.setLowPowerModeCallback(enter_low_power_mode);
    gaspetto_box.init(StateId::IDLE);
    /* Set up ISR for button press simulation. */
    attachInterrupt(digitalPinToInterrupt(PB_0), ISR, RISING);
}

void loop()
{
    gaspetto_box.work();
}

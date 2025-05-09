#include "Arduino.h"
#include "EventQueue.h"
#include "GaspettoBox.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include <atomic>

#ifndef ARDUINO
std::atomic<bool> lowPowerMode;
#endif

// Pin Definitions
// const uint8_t adcPins[4] = {PB0, PB1, PB10, PB11}; // ADC Channels
// const uint8_t groupPins[20] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB12,
// PB13,
//                                PB14, PB15, PC13, PC14, PC15, PA8, PA9, PA10,
//                                PA11, PA12}; // GPIO Pins for GND control

IdleState idleState;
ProcessingState processingState;
EventQueue eventQueue;
GaspettoBox gaspetto_box(&idleState, &processingState, &eventQueue, StateId::IDLE);

// Button press simulation thread
void ISR(void) {
  Event evt = getEvent();
  gaspetto_box.debounceAndEnqueue(evt, millis());
}

void setup() {}

void loop() { gaspetto_box.processNextEvent(); }
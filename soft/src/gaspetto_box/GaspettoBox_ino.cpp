#include "Arduino.h"
#include "EventQueue.h"
#include "GaspettoBox.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include <atomic>

// Pin Definitions
// const uint8_t adcPins[4] = {PB0, PB1, PB10, PB11}; // ADC Channels
// const uint8_t groupPins[20] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB12,
// PB13,
//                                PB14, PB15, PC13, PC14, PC15, PA8, PA9, PA10,
//                                PA11, PA12}; // GPIO Pins for GND control

IdleState idleState;
ProcessingState processingState;
EventQueue eventQueue;
GaspettoBox gaspetto_box(&idleState, &processingState, &eventQueue,
                         StateId::IDLE);

// Button press simulation thread
void ISR(void) {
#ifndef ARDUINO
  Event evt = getEvent();
  gaspetto_box.debounceAndEnqueue(evt, millis());
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.println("Gaspetto Box Initialized");
  Serial.println("Starting up...\n");
  /* Initialize the GaspettoBox state machine. */
  gaspetto_box.Init();
  /* Set up ISR for button press simulation. */
  attachInterrupt(digitalPinToInterrupt(PB0), ISR, RISING);
}

void loop() { gaspetto_box.processNextEvent(); }
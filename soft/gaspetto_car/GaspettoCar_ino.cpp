#include "Arduino.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "IdleState.h"
#include "ProcessingState.h"

#include <cstdlib>
#include <ctime> // Required for std::time
#include <atomic> // Required for std::atomic

#ifndef ARDUINO
std::atomic<bool> lowPowerMode;
#endif

IdleState idleState;
ProcessingState processingState;
EventQueue eventQueue;
GaspettoCar gaspetto(&idleState, &processingState, &eventQueue, StateId::IDLE);

#ifndef ARDUINO
void enqueue_random_commands(const uint8_t num_events) {
  srand(std::time(nullptr));

  for (uint8_t i = 0; i < num_events; ++i) {
    const CommandId command = static_cast<CommandId>(
        rand() % static_cast<int>(CommandId::MAX_COMMAND_ID));
    const Event event(EventId::NRF_IRQ,
                      command); // Random event

    gaspetto.postEvent(event);
  }
}
#endif

void nrf_ISR(void) {
#ifndef ARDUINO
  enqueue_random_commands(1);
#endif
}

void setup() {
  gaspetto.Init();
}

void loop() { gaspetto.processNextEvent(); }
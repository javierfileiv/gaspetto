#include "Arduino.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "IdleState.h"
#include "ProcessingState.h"

#include <atomic> /* Required for std::atomic*/
#include <cstdlib>
#include <ctime> /* Required for std::time*/

#ifndef ARDUINO
std::atomic<bool> lowPowerMode;
#endif

IdleState idleState;
ProcessingState processingState;
EventQueue eventQueue;
GaspettoCar gaspetto_car(&idleState, &processingState, &eventQueue,
                         StateId::IDLE);

#ifndef ARDUINO
void enqueue_random_commands(const uint8_t num_events) {
  srand(std::time(nullptr));

  for (uint8_t i = 0; i < num_events; ++i) {
    const CommandId command = static_cast<CommandId>(
        rand() % static_cast<int>(CommandId::MAX_COMMAND_ID));
    const Event event(EventId::NRF_IRQ, command); /* Random event*/
    gaspetto_car.postEvent(event);
  }
}
#endif

void ISR(void) {
#ifndef ARDUINO
  Event evt = getEvent();
  gaspetto_car.postEvent(evt);
#else

#endif
}

void setup() { gaspetto_car.Init(); }

void loop() { gaspetto_car.processNextEvent(); }
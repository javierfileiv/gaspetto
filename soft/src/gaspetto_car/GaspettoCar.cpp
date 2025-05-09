#include "GaspettoCar.h"
#include <iostream>

GaspettoCar::GaspettoCar(State *idle, State *running, EventQueue *queue,
                         StateId initial_state)
    : eventQueue(queue) {
#ifndef ARDUINO
  lowPowerMode = false; // Initialize lowPowerMode to false
#endif

  /* Initialize states. */
  states[static_cast<int>(StateId::IDLE)] = idle;
  states[static_cast<int>(StateId::PROCESSING)] = running;

  /* Set up state machine references. */
  for (int i = 0; i < static_cast<int>(StateId::MAX_STATE_ID); i++) {
    if (states[i] != nullptr) {
      states[i]->setMachine(this);
    }
  }
  currentStateId = initial_state;
}

void GaspettoCar::Init(void) {
  states[static_cast<int>(currentStateId)]->enter();
}

void GaspettoCar::UpdateState(StateId &newStateId) {
  State *currentState = states[static_cast<int>(currentStateId)];

  currentState->exit();
  currentState = states[static_cast<int>(newStateId)];
  currentState->enter();
}

void GaspettoCar::enqueue_random_commands(const uint8_t num_events) {
  std::srand(time(nullptr));

  for (uint8_t i = 0; i < num_events; ++i) {
    Event event = {EventId::NRF_IRQ,
                   static_cast<CommandId>(rand() % 4)}; // Random event

    postEvent(event);
    lowPowerMode = false; // Wake the system
  }
}

int GaspettoCar::postEvent(Event evt) {
  if (eventQueue->IsFull()) {
    std::cout << "Event queue is full, cannot post event.\n";
    return -1;
  }
  eventQueue->enqueue(evt);
  return 0;
}

void GaspettoCar::processNextEvent(void) {
  if (!eventQueue->IsEmpty()) {
    Event evt;

    State *currentState = states[static_cast<int>(currentStateId)];
    eventQueue->dequeue(evt);
    currentState->processEvent(evt);
  }
}

void GaspettoCar::transitionTo(StateId newStateId) {
  State *currentState = states[static_cast<int>(currentStateId)];

  currentState->exit();
  currentStateId = newStateId;
  currentState = states[static_cast<int>(currentStateId)];
  currentState->enter();
}

void GaspettoCar::enterLowPowerMode(void) {
#ifndef ARDUINO
  std::cout << "Entering low-power mode...\n";
  lowPowerMode = true;
  while (lowPowerMode) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)); // Simulate low-power sleep
  }
#endif
}

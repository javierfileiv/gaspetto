#include "GaspettoBox.h"
#include <iostream>

GaspettoBox::GaspettoBox(State *idle, State *running, EventQueue *queue,
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

void GaspettoBox::Init(void) {
  states[static_cast<int>(currentStateId)]->enter();
}

void GaspettoBox::UpdateState(StateId &newStateId) {
  State *currentState = states[static_cast<int>(currentStateId)];

  currentState->exit();
  currentState = states[static_cast<int>(newStateId)];
  currentState->enter();
}

int GaspettoBox::postEvent(Event evt) {
  if (eventQueue->IsFull()) {
    std::cout << "Event queue is full, cannot post event.\n";
    return -1;
  }
  eventQueue->enqueue(evt);
  return 0;
}

void GaspettoBox::transitionTo(StateId newStateId) {
  State *currentState = states[static_cast<int>(currentStateId)];

  currentState->exit();
  currentStateId = newStateId;
  currentState = states[static_cast<int>(currentStateId)];
  currentState->enter();
}
// void GaspettoBox::handleEvent(Event event) {
//   if (event == BUTTON_PRESSED) {
//     if (currentState == IDLE) {
//       std::cout << "Button pressed. Transitioning to PROCESSING state.\n";
//       currentState = PROCESSING;
//     } else {
//       std::cout << "Button pressed, but system is not in IDLE state.\n";
//     }
//   }
// }
void GaspettoBox::enterLowPowerMode(void) {
#ifndef ARDUINO
  std::cout << "Entering low-power mode...\n";
  lowPowerMode = true;
  while (lowPowerMode) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)); // Simulate low-power sleep
  }
#endif
}

void GaspettoBox::processNextEvent(void) {
  if (!eventQueue->IsEmpty()) {
    Event evt;

    State *currentState = states[static_cast<int>(currentStateId)];
    eventQueue->dequeue(evt);
    currentState->processEvent(evt);
  }
}
// {
//   // Process events from the queue
//   Event event;
//   while (getEventQueue().dequeue(event)) {
//     handleEvent(event);
//   }

//   // Perform actions based on the current state
//   switch (currentState) {
//   case IDLE:
//     enterLowPowerMode();
//     break;

//   case PROCESSING:
//     for (int row = 0; row < 3; ++row) {
//       std::cout << "Processing row " << row << "...\n";
//       std::cout << "Scanning row " << row << "...\n";
//       std::this_thread::sleep_for(
//           std::chrono::duration<int>(1)); // Simulate some processing work
//       std::cout << "Sending row " << row << "...\n";
//     }
//     currentState = DONE; // Transition to DONE state
//     break;

//   case DONE:
//     std::cout << "Processing complete. Returning to IDLE state.\n";
//     currentState = IDLE;
//     break;
//   }
// }

void GaspettoBox::debounceAndEnqueue(Event &evt, unsigned long currentTime) {
#ifdef ARDUINO
  if (currentTime - lastDebounceTime > debounceDelay) {
    lastDebounceTime = currentTime;
    if (!getEventQueue().full()) {
      getEventQueue().enqueue(evt);
      std::cout << "Exiting low-power mode...\n";
      lowPowerMode = false; // Wake the system
    } else {
      std::cout << "Event queue is full! Unable to enqueue event.\n";
    }
  }
#else
  if (!getEventQueue().IsFull()) {
    postEvent(evt);
    std::cout << "Button pressed. Exiting low-power mode...\n";
  } else {
    std::cout << "Event queue is full! Unable to enqueue event.\n";
  }
#endif
}

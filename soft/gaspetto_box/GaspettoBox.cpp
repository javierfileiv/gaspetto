#include "GaspettoBox.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>

GaspettoBox::GaspettoBox(EventQueue &queue)
    : ActiveObject(queue), lowPowerMode(false) {}

void GaspettoBox::handleEvent(Event event) {
  if (event == BUTTON_PRESSED) {
    if (currentState == IDLE) {
      std::cout << "Button pressed. Transitioning to PROCESSING state.\n";
      currentState = PROCESSING;
    } else {
      std::cout << "Button pressed, but system is not in IDLE state.\n";
    }
  }
}

void GaspettoBox::setLowPorerModeReference(std::atomic<bool> &lowPower) {
  lowPowerMode.store(lowPower.load());
}
void GaspettoBox::process() {
  // Process events from the queue
  Event event;
  while (getEventQueue().dequeue(event)) {
    handleEvent(event);
  }

  // Perform actions based on the current state
  switch (currentState) {
  case IDLE:
    enterLowPowerMode();
    break;

  case PROCESSING:
    for (int row = 0; row < 3; ++row) {
      std::cout << "Processing row " << row << "...\n";
      std::cout << "Scanning row " << row << "...\n";
      std::this_thread::sleep_for(
          std::chrono::duration<int>(1)); // Simulate some processing work
      std::cout << "Sending row " << row << "...\n";
    }
    currentState = DONE; // Transition to DONE state
    break;

  case DONE:
    std::cout << "Processing complete. Returning to IDLE state.\n";
    currentState = IDLE;
    break;
  }
}

void GaspettoBox::enterLowPowerMode() {
  std::cout << "Entering low-power mode...\n";
  lowPowerMode = true;
  while (lowPowerMode) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)); // Simulate low-power sleep
  }
}

void GaspettoBox::debounceAndEnqueue(unsigned long currentTime) {
#ifdef ARDUINO
  if (currentTime - lastDebounceTime > debounceDelay) {
    lastDebounceTime = currentTime;
    if (!getEventQueue().full()) {
      getEventQueue().enqueue(BUTTON_PRESSED);
      std::cout << "Exiting low-power mode...\n";
      lowPowerMode = false; // Wake the system
    } else {
      std::cout << "Event queue is full! Unable to enqueue event.\n";
    }
  }
#else
  if (!getEventQueue().full()) {
    getEventQueue().enqueue(BUTTON_PRESSED);
    std::cout << "Button pressed. Exiting low-power mode...\n";
    lowPowerMode = false; // Wake the system
  } else {
    std::cout << "Event queue is full! Unable to enqueue event.\n";
  }
#endif
}

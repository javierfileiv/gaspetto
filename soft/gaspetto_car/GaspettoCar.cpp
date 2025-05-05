#include "GaspettoCar.h"
#include "EventQueue.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>

GaspettoCar::GaspettoCar(EventQueue &queue)
    : ActiveObject(queue), lowPowerMode(false) {}

void GaspettoCar::setLowPowerModeReference(std::atomic<bool> &lowPower) {
  lowPowerMode.store(lowPower.load());
}
void GaspettoCar::handleEvent(Event event) {};

void GaspettoCar::process() {
  // Process events from the queue
  EventData event;
  while (1) {
    // Perform actions based on the current state
    switch (currentState) {
    case IDLE:
      enterLowPowerMode();
      break;
    case PROCESSING:
      switch (event.command) {
      case MOTOR_FORWARD:
        std::cout << "Moving forward...\n";
        break;
      case MOTOR_BACKWARD:
        std::cout << "Moving backward...\n";
        break;
      case MOTOR_RIGHT:
        std::cout << "Turning right...\n";
        break;
      case MOTOR_LEFT:
        std::cout << "Turning left...\n";
        break;
      default:
        std::cout << "Unknown event in PROCESSING state.\n";
        break;
      }
    }
    if (getEventQueue().empty()) {
      std::cout << "No more events. Transitioning to IDLE state.\n";
      currentState = IDLE;
    } else {
      getEventQueue().dequeue(event);
    }
  }
}
void GaspettoCar::enterLowPowerMode() {
  std::cout << "Entering low-power mode...\n";
  lowPowerMode = true;
  while (lowPowerMode) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100)); // Simulate low-power sleep
  }
}

void GaspettoCar::enqueue_random_events(const uint8_t num_events) {
  currentState = PROCESSING;
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
  // Seed the random number generator
  std::srand(std::time(nullptr));
  for (uint8_t i = 0; i < num_events; ++i) {
    EventData event = {NRF_IRQ,
                       static_cast<Command>(rand() % 4)}; // Random event
    if (!getEventQueue().full()) {
      getEventQueue().enqueue(event);
      std::cout << "Enqueued event: " << event.command << "\n";
    } else {
      std::cout << "Event queue is full! Unable to enqueue event.\n";
    }
    lowPowerMode = false; // Wake the system
}
#endif
}
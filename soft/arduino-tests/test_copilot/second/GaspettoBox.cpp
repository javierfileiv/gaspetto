#include "ActiveObject.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <ostream>
#include <thread>

// Define states
enum State { IDLE, PROCESSING, DONE };

// Active Object class encapsulates the main logic
class GaspettoBox : public ActiveObject {
private:
  State currentState = IDLE;
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50; // 50ms debounce

  std::atomic<bool> &lowPowerMode; // Reference to low-power mode flag

public:
  GaspettoBox(EventQueue &queue, std::atomic<bool> &lowPower)
      : ActiveObject(queue), lowPowerMode(lowPower) {}

  void handleEvent(Event event) {
    if (event == BUTTON_PRESSED) {
      if (currentState == IDLE) {
        std::cout << "Button pressed. Transitioning to PROCESSING state.\n";
        currentState = PROCESSING;
      } else {
        std::cout << "Button pressed, but system is not in IDLE state.\n";
      }
    }
  }

  void process() {
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
            std::chrono::duration<int>(4)); // Simulate some processing work
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

  void enterLowPowerMode() {
    std::cout << "Entering low-power mode...\n";
    lowPowerMode = true;
    while (lowPowerMode) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(100)); // Simulate low-power sleep
    }
    std::cout << "Waking up from low-power mode...\n";
  }

  void debounceAndEnqueue(unsigned long currentTime) {
    if (currentTime - lastDebounceTime > debounceDelay) {
      lastDebounceTime = currentTime;
      if (!getEventQueue().full()) {
        getEventQueue().enqueue(BUTTON_PRESSED);
        lowPowerMode = false; // Wake the system
      } else {
        std::cout << "Event queue is full! Unable to enqueue event.\n";
      }
    }
  }
};

// Global variables
std::atomic<unsigned long> millisCounter(0); // Simulated millis()
EventQueue eventQueue;                       // Shared event queue
std::atomic<bool> running(true);             // Flag to stop threads
std::atomic<bool> lowPowerMode(false);       // Simulates low-power mode

// Simulated millis function
unsigned long millis() { return millisCounter.load(); }

// Button press simulation thread
void buttonThread(GaspettoBox &activeObject) {
  while (running) {
    std::this_thread::sleep_for(
        std::chrono::seconds(2)); // Simulate button press every 2 seconds
    std::cout << "Simulating button press...\n";
    activeObject.debounceAndEnqueue(millis());
  }
}

// Millis simulation thread
void millisThread() {
  while (running) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1)); // Increment every millisecond
    millisCounter.fetch_add(1);
  }
}

int main() {
  // Create the ActiveObject instance
  GaspettoBox activeObject(eventQueue, lowPowerMode);

  // Start the simulation threads
  std::thread millisSim(millisThread);
  std::thread buttonSim(buttonThread, std::ref(activeObject));

  // Main loop
  while (running) {
    activeObject.process();
    std::this_thread::sleep_for(std::chrono::milliseconds(
        10)); // Add a small delay to prevent CPU overuse
  }

  // Stop the threads
  running = false;
  millisSim.join();
  buttonSim.join();

  return 0;
}
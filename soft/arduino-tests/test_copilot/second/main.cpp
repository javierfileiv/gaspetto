#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

// Global variables
std::atomic<unsigned long> millisCounter(0); // Simulated millis()
EventQueue eventQueue;                       // Shared event queue
std::atomic<bool> running(true);             // Flag to stop threads
std::atomic<bool> lowPowerMode(false);       // Simulates low-power mode

// Simulated millis function
unsigned long millis() { return millisCounter.load(); }

// Button press simulation thread
void buttonThread(ActiveObject &activeObject) {
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
  ActiveObject activeObject(eventQueue, lowPowerMode);

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
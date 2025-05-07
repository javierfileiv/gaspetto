#include "../GaspettoCar.h"
#include "GaspettoCar_ino.h"
#include <atomic>
#include <iostream>
#include <thread>
#include <unistd.h>

extern "C" {
#include <termios.h>
}
// gloabl variable from ino
extern GaspettoCar gaspetto;

// Global variables
std::atomic<unsigned long> millisCounter(0);
std::atomic<bool> running(true);
extern std::atomic<bool> lowPowerMode;

// Simulated millis function
unsigned long millis(void) { return millisCounter.load(); }

// Millis simulation thread
static void emu_millisThread() {
  while (running) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1)); // Increment every millisecond
    millisCounter.fetch_add(1);
  }
}

static void keyboardInput(void) {
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char ch;
  bool keyPressed = false;
  Event event;
  while (true) {
    if (read(STDIN_FILENO, &ch, 1) > 0) {
      switch (ch) {
      case 'I':
      case 'i':
        nrf_ISR();                 // Simulate NRF IRQ
        lowPowerMode.store(false); /* Wake Up the system. */
        break;
      case 'q':
      case 'Q':
        running = false; // Stop the program
        break;
      case 'F':
      case 'f':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_FORWARD};
        gaspetto.postEvent(event);
        lowPowerMode.store(false); // Wake the system
        break;
      case 'b':
      case 'B':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_BACKWARD};
        gaspetto.postEvent(event);
        lowPowerMode.store(false); // Wake the system
        break;
      case 'l':
      case 'L':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_LEFT};
        gaspetto.postEvent(event);
        lowPowerMode.store(false); // Wake the system
        break;
      case 'r':
      case 'R':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_RIGHT};
        gaspetto.postEvent(event);
        lowPowerMode.store(false); // Wake the system
        break;
      case 's':
      case 'S':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_STOP};
        gaspetto.postEvent(event);
        lowPowerMode.store(false); // Wake the system
        break;
      }
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(250)); // Polling delay
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore terminal settings
}

void enterLowPowerMode(void) {
  std::cout << "Entering low-power mode...\n";
#ifndef ARDUINO
  lowPowerMode.store(true);
  while (lowPowerMode.load()) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds{100}); // Simulate low-power sleep
  }
#endif
}

int main() {
  std::cout << "Starting simulation...\n";
  // Start the simulation threads
  std::thread millisSim(emu_millisThread);
  std::thread keyboardSim(keyboardInput);
  // Setuo the system
  setup();
  // Main loop
  while (running) {
    loop();
    std::this_thread::sleep_for(std::chrono::milliseconds(
        10)); // Add a small delay to prevent CPU overuse
  }

  // Stop the threads
  running = false;
  millisSim.join();
  keyboardSim.join();
  return 0;
}
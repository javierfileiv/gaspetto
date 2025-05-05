#include "EventQueue.h"
#include "GaspettoCar.h"
#include "GaspettoCar_ino.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <unistd.h> // For read() on Linux

extern "C" {
#include <termios.h>
}
// gloabl variable from ino
extern GaspettoCar gaspetto;

// Global variables
std::atomic<unsigned long> millisCounter(0); // Simulated millis()
std::atomic<bool> running(true);             // Flag to stop threads

// Simulated millis function
unsigned long millis(void) { return millisCounter.load(); }

// Button press simulation thread
// static void emu_buttonThread(void) {
//   while (running) {
//     std::this_thread::sleep_for(
//         std::chrono::seconds(2)); // Simulate button press every 2 seconds
//     std::cout << "Simulating button press...\n";
//     buttonISR();
//   }
// }

// Millis simulation thread
static void emu_millisThread(void) {
  while (running) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1)); // Increment every millisecond
    millisCounter.fetch_add(1);
  }
}

extern EventQueue eventQueue; // Shared event queue

void keyboardInput(void) {
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char ch;
  bool keyPressed = false;
  EventData event;
  while (true) {
    if (read(STDIN_FILENO, &ch, 1) > 0) {
      switch (ch) {
      case 'I':
      case 'i':
        nrf_ISR(); // Simulate NRF IRQ
        break;
      case 'q':
      case 'Q':
        running = false; // Stop the program
        break;
      case 'F':
      case 'f':
        event = {NRF_IRQ, MOTOR_FORWARD};
        eventQueue.enqueue(event);
        break;
      case 'b':
      case 'B':
        event = {NRF_IRQ, MOTOR_BACKWARD};
        eventQueue.enqueue(event);
        break;
      case 'l':
      case 'L':
        event = {NRF_IRQ, MOTOR_LEFT};
        eventQueue.enqueue(event);
        break;
      case 'r':
      case 'R':
        event = {NRF_IRQ, MOTOR_RIGHT};
        eventQueue.enqueue(event);
        break;
      }
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(250)); // Polling delay
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore terminal settings
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
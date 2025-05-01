#include "GaspettoBox.h"
#include "GaspettoBox_ino.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <unistd.h> // For read() on Linux

extern "C" {
#include <termios.h>
}
// gloabl variable from ino
extern GaspettoBox gaspetto;

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

void keyboardInput(void) {
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char ch;
  bool keyPressed = false;

  while (true) {
    if (read(STDIN_FILENO, &ch, 1) > 0) {
      if (ch == 'b' || ch == 'B') { // Check if 'B' key is pressed
        // if (!keyPressed) {
        //   keyPressed = true;
          buttonISR();
          //   eventQueue.enqueue(BUTTON_PRESSED);
        // }
      }
//     else {
//         if (keyPressed) {
//           keyPressed = false;
//           //   eventQueue.enqueue(BUTTON_RELEASED);
//         }
//       }
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(250)); // Polling delay
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore terminal settings
}

int main() {

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
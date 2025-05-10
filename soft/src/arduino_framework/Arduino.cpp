#include "Event.h"
#include "GaspettoCar_ino.h"
#include "Serial.h"
#include <atomic>
#include <iostream>
#include <thread>
#include <unistd.h>

extern "C" {
#include <termios.h>
}

/*  Global variables. */
std::atomic<unsigned long> millisCounter(0);
std::atomic<bool> running(true);
std::atomic<bool> lowPowerMode;
void (*userFunc_)(void) = nullptr;
Event event;
Event evt_copy;
/*  Simulated millis function. */
unsigned long millis(void) { return millisCounter.load(); }

/*  Millis simulation thread. */
static void emu_millisThread() {
  while (running) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1)); /*  Increment every millisecond. */
    millisCounter.fetch_add(1);
  }
}

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void),
                     uint8_t mode) {
  userFunc_ = userFunc;
}

Event getEvent(void) {
  evt_copy = event;
  return evt_copy;
}

static void keyboardInput(void) {
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char ch;
  bool keyPressed = false;
  while (true) {
    if (read(STDIN_FILENO, &ch, 1) > 0) {
      event = {EventId::NONE, CommandId::NONE}; /*  Reset event. */
      switch (ch) {
      case 'q':
      case 'Q':
        running = false; /*  Stop the program. */
        break;
      case 'F':
      case 'f':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_FORWARD};
        break;
      case 'b':
      case 'B':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_BACKWARD};
        break;
      case 'l':
      case 'L':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_LEFT};
        break;
      case 'r':
      case 'R':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_RIGHT};
        break;
      case 's':
      case 'S':
        event = {EventId::NRF_IRQ, CommandId::MOTOR_STOP};
        break;
      case 'p':
      case 'P':
        event = {EventId::BUTTON_PRESSED, CommandId::NONE};
        break;
      }
      ISR();
      lowPowerMode.store(false);
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(250)); /*  Polling delay. */
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*  Restore terminal settings. */
}

void enterLowPowerMode(void) {
  Serial.println("Entering low-power mode...\n");
#ifndef ARDUINO
  lowPowerMode.store(true);
  while (lowPowerMode.load()) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds{100}); /*  Simulate low-power sleep. */
  }
#endif
}

int main() {
  Serial.println("Starting simulation...\n");
  /*  Start the simulation threads. */
  std::thread millisSim(emu_millisThread);
  std::thread keyboardSim(keyboardInput);
  /*  Setuo the system. */
  setup();
  /*  Main loop. */
  while (running) {
    loop();
    std::this_thread::sleep_for(std::chrono::milliseconds(
        10)); /*  Add a small delay to prevent CPU overuse. */
  }

  /*  Stop the threads. */
  running = false;
  millisSim.join();
  keyboardSim.join();
  return 0;
}
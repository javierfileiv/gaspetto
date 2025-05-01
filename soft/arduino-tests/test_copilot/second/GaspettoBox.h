#include "ActiveObject.h"
#include <atomic>
#include <iostream>

// Define states
enum State { IDLE, PROCESSING, DONE };

// Active Object class encapsulates the main logic
class GaspettoBox : public ActiveObject {
private:
  State currentState = IDLE;
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50; // 50ms debounce

  std::atomic<bool> lowPowerMode; // Reference to low-power mode flag

public:
  GaspettoBox(EventQueue &queue);

  void handleEvent(Event event) override;

  void process() override;

  void enterLowPowerMode();

  void debounceAndEnqueue(unsigned long currentTime);

  void setLowPoerModeReference(std::atomic<bool> &lowPower);
  };
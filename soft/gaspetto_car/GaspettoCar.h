#include "ActiveObject.h"
#include <atomic>
#include <iostream>

// Define states
enum State {
  IDLE,
  PROCESSING,
};

// Active Object class encapsulates the main logic
class GaspettoCar : public ActiveObject {
private:
  State currentState = IDLE;
  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50; // 50ms debounce

  std::atomic<bool> lowPowerMode; // Reference to low-power mode flag

public:
  GaspettoCar(EventQueue &queue);

  void handleEvent(Event event) override;

  void process() override;

  void enterLowPowerMode();

  void enqueue_random_events(const uint8_t num_events);

  void setLowPowerModeReference(std::atomic<bool> &lowPower);
};
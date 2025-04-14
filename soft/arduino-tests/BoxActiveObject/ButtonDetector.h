// ButtonDetector.h
#ifndef BUTTON_DETECTOR_H
#define BUTTON_DETECTOR_H

#include "MessageButton.h" // Include para MessageButton
#include "MessageQueue.h"
#include <Arduino.h>
#include <stdint.h>

class ButtonDetector {
public:
  ButtonDetector(uint8_t buttonPin, MessageQueue &messageQueue,
                 unsigned long debounceDelayMs = 50);
  void checkButton();

private:
  uint8_t buttonPin_;
  MessageQueue &messageQueue_;
  unsigned long debounceDelayMs_;
  bool lastButtonState_ = HIGH;
  unsigned long lastDebounceTime_ = 0;
};

#endif
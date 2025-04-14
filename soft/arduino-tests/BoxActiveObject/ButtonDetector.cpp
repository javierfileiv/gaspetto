// ButtonDetector.cpp
#include "ButtonDetector.h"
#include "MessageButton.h" // Include para crear MessageButton

ButtonDetector::ButtonDetector(uint8_t buttonPin, MessageQueue &messageQueue,
                               unsigned long debounceDelayMs)
    : buttonPin_(buttonPin), messageQueue_(messageQueue),
      debounceDelayMs_(debounceDelayMs) {
  pinMode(buttonPin_, INPUT_PULLUP);
}

void ButtonDetector::checkButton() {
  bool currentButtonState = digitalRead(buttonPin_);
  unsigned long currentTime = millis();

  if (currentButtonState != lastButtonState_) {
    lastDebounceTime_ = currentTime;
  }

  if ((currentTime - lastDebounceTime_) > debounceDelayMs_) {
    if (currentButtonState == LOW && lastButtonState_ == HIGH) {
      MessageButton *msg = messageQueue_.allocate<MessageButton>(buttonPin_);
      if (msg != nullptr) {
        messageQueue_.enqueue(msg);
      } else {
        Serial.println("Pool de MessageButton lleno!");
      }
    }
    lastButtonState_ = currentButtonState;
  }
}
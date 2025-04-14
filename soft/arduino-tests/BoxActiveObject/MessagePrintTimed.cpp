#include "MessagePrintTimed.h"
#include <Arduino.h>
#include <string.h>

MessagePrintTimed::MessagePrintTimed(const char *text, unsigned long delayMs)
    : delayMs_(delayMs) {
  strncpy(text_, text, sizeof(text_) - 1);
  text_[sizeof(text_) - 1] = '\0';
}

void MessagePrintTimed::execute() {
  if (!delayed_) {
    startTime_ = millis();
    delayed_ = true;
  }
  if (!printed_ && (millis() - startTime_ >= delayMs_)) {
    Serial.print("[DELAYED] ");
    Serial.println(text_);
    printed_ = true; // Marcar como impreso
  }
}
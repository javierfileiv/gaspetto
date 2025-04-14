#include "MessagePrint.h"
#include <Arduino.h>
#include <string.h>

MessagePrint::MessagePrint(const char* text) {
  strncpy(text_, text, sizeof(text_) - 1);
  text_[sizeof(text_) - 1] = '\0'; // Asegurar null termination
}

void MessagePrint::execute() {
  Serial.println(text_);
}
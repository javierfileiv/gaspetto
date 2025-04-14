#include "MessageButton.h"
#include <Arduino.h>
#include <stdint.h>

MessageButton::MessageButton(uint8_t pin) : pin_(pin) {
  snprintf(payload_, sizeof(payload_), "BUTTON %d", pin_);
}

void MessageButton::execute() {
  Serial.print("Botón presionado en el pin: ");
  Serial.println(pin_);
}

const char *MessageButton::getPayload() const { return payload_; }
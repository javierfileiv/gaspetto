#include "MessageNrfRx.h"
#include "MessagePrint.h"
#include <Arduino.h>
#include <string.h>

MessageNrfRx::MessageNrfRx(const char* receivedData) {
  strncpy(receivedData_, receivedData, sizeof(receivedData_) - 1);
  receivedData_[sizeof(receivedData_) - 1] = '\0';
}

void MessageNrfRx::execute() {
  Serial.print("NRF Recibido: ");
  Serial.println(receivedData_);
  // Aquí podrías añadir lógica para procesar los datos recibidos
}

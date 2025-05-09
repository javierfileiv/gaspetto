#include "ProcessingState.h"
#include "Event.h"
#include "GaspettoBox.h"
#include <cassert>
#include <iostream>

void ProcessingState::enter() {
  for (int row = 0; row < 3; ++row) {
    Serial.print("Scanning row ");
    Serial.print(row);
    Serial.println("...\n");
#ifndef ARDUINO
    std::this_thread::sleep_for(
        std::chrono::duration<int>(1)); // Simulate some processing work
#else
    delay(1000); // Simulate some processing work
#endif
    Serial.print("Processing row ");
    Serial.print(row);
    Serial.println("...\n");
    Serial.print("Sending row ");
    Serial.print(row);
    Serial.println("...\n");
  }
  state_machine->transitionTo(StateId::IDLE);
}
#include "ProcessingState.h"
#include "GaspettoCar.h"
#include <cassert>
#include <iostream>

void ProcessingState::processEvent(Event evt) {
  switch (evt.getCommand()) {
  case CommandId::MOTOR_FORWARD:
    Serial.println("Moving forward...\n");
    break;
  case CommandId::MOTOR_BACKWARD:
    Serial.println("Moving backward...\n");
    break;
  case CommandId::MOTOR_RIGHT:
    Serial.println("Turning right...\n");
    break;
  case CommandId::MOTOR_LEFT:
    Serial.println("Turning left...\n");
    break;
  case CommandId::MOTOR_STOP:
    Serial.println("Stopping...\n");
    state_machine->transitionTo(StateId::IDLE);
    break;
  default:
    Serial.println("Unknown event in PROCESSING state.\n");
    assert(true);
    break;
  }
}
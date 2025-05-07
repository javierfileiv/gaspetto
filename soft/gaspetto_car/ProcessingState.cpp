#include "ProcessingState.h"
#include "GaspettoCar.h"
#include <cassert>
#include <iostream>

void ProcessingState::processEvent(Event evt) {
  switch (evt.getCommand()) {
  case CommandId::MOTOR_FORWARD:
    std::cout << "Moving forward...\n";
    break;
  case CommandId::MOTOR_BACKWARD:
    std::cout << "Moving backward...\n";
    break;
  case CommandId::MOTOR_RIGHT:
    std::cout << "Turning right...\n";
    break;
  case CommandId::MOTOR_LEFT:
    std::cout << "Turning left...\n";
    break;
  case CommandId::MOTOR_STOP:
    std::cout << "Stopping...\n";
    car_state_machine->transitionTo(StateId::IDLE);
    break;
  default:
    std::cout << "Unknown event in PROCESSING state.\n";
    assert(true);
    break;
  }
}
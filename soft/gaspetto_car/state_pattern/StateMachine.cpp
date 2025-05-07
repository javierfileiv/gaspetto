#include "StateMachine.h"
#include "EventQueue.h"

StateMachine::StateMachine()
    : idleState(eventQueue), processingState(eventQueue),
      currentState(&idleState) {
  currentState->enter();
}

void StateMachine::ProcessEvent(EventData event) {
  currentState->processEvent(event);
  if (event.command == Command::MOTOR_FORWARD ||
      event.command == Command::MOTOR_BACKWARD ||
      event.command == Command::MOTOR_RIGHT ||
      event.command == Command::MOTOR_LEFT) {
    UpdateState(processingState);
  } else {
    UpdateState(idleState);
  }
}

void StateMachine::UpdateState(State &newState) {
  currentState->exit();
  currentState = &newState;
  currentState->enter();
}

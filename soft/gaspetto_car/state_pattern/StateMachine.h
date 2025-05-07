#include "State.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "EventQueue.h"
#include <iostream>

class StateMachine {
public:
  StateMachine();

  void ProcessEvent(EventData event);

private:
  void UpdateState(State &newState);

private:
  EventQueue eventQueue;
  State *currentState;
  IdleState idleState;
  ProcessingState processingState;
};
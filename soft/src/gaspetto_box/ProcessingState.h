#pragma once

#include "StateMachine.h"
#include "State.h"

class Event;

class ProcessingState : public State {
public:
  void enter() override;
};
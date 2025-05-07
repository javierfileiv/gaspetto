#pragma once

#include "State.h"

class IdleState : public State {
public:
  IdleState(EventQueue &queue) : State(queue) {}
  void enter() override;

  void exit() override;

  void processEvent(EventData event) override;
};
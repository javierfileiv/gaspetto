#pragma once

#include "State.h"

class ProcessingState : public State {
public:
  ProcessingState(EventQueue &queue) : State(queue) {}
  void enter() override;

  void exit() override;

  void processEvent(EventData event) override;
};
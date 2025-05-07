#pragma once

#include "EventQueue.h"

class State {
public:
  State(EventQueue &queue) : eventQueue(queue) {}
  virtual void enter() = 0;
  virtual void exit() = 0;
  virtual void processEvent(EventData) = 0;

private:
  EventQueue &eventQueue;
};


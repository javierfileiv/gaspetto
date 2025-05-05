#pragma once

#include "EventQueue.h"
#include <atomic>

class ActiveObject {

public:
  ActiveObject(EventQueue &queue) : eventQueue(queue) {};
  virtual void handleEvent(Event event) = 0;
  virtual void process() = 0;
  EventQueue& getEventQueue() { return eventQueue; }

private:
  EventQueue &eventQueue; // Reference to the shared event queue
};
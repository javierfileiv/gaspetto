#pragma once

#include "Event.h" // Assuming you have your ActiveObject definition
#include <cstdint>

#define MAX_TIMED_EVENT_NODES 10 // Define a maximum number of scheduled events

struct TimedEventNode {
  uint32_t triggerTimeMs;
  Event event;
  int nextIndex; // Index of the next node in the array (-1 for null)

  TimedEventNode() : triggerTimeMs(0), nextIndex(-1) {}
};

class TimeredEventQueue {
public:
  TimeredEventQueue();
  bool scheduleEvent(uint32_t timeMs,
                     Event event); // Schedule based on absolute time
  bool scheduleEventDelayed(uint32_t delayMs,
                            Event event); // Schedule based on delay
  void processEvents();
  void clear(); // Reset the queue

private:
  TimedEventNode eventNodes_[MAX_TIMED_EVENT_NODES];
  int headIndex_; // Index of the first event in the sorted list (-1 if empty)
  int freeListHead_; // Index of the first free node in the array (-1 if full)

  int allocateNode();
  void freeNode(int index);
};

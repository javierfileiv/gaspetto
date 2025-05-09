#pragma once

#include "StateMachine.h"
#include <cstddef> // For size_t

#define QUEUE_SIZE 10 // Define the size of the queue

class EventQueue {
public:
  // Enqueue an event
  bool enqueue(Event &evt);

  // Dequeue an event
  bool dequeue(Event &evt);

  // Check if the queue is empty
  bool IsEmpty() const;

  // Check if the queue is full
  bool IsFull() const;

  // Get the current size of the queue
  size_t GetSize() const;

private:
  const int capacity = QUEUE_SIZE;
  Event events[QUEUE_SIZE];
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;
};

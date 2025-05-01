#pragma once

#include <cstddef> // For size_t

// Define events
enum Event { BUTTON_PRESSED };
class EventQueue {
private:
  static const size_t capacity = 10; // Fixed size of the queue
  Event events[capacity];            // Preallocated array for events
  size_t head = 0;                   // Index of the first element
  size_t tail = 0;                   // Index of the next available slot
  size_t count = 0;                  // Number of elements in the queue

public:
  // Enqueue an event
  bool enqueue(Event event);

  // Dequeue an event
  bool dequeue(Event &event);

  // Check if the queue is empty
  bool empty() const;

  // Check if the queue is full
  bool full() const;

  // Get the current size of the queue
  size_t size() const;
};

#pragma once

#include <cstddef> // For size_t

// Define events
enum Event { NRF_IRQ };
enum Command { MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_RIGHT, MOTOR_LEFT };
struct EventData {
  Event event;
  Command command;
};

class EventQueue {
private:
  static const size_t capacity = 10; // Fixed size of the queue
  EventData events[capacity];            // Preallocated array for events
  size_t head = 0;                   // Index of the first element
  size_t tail = 0;                   // Index of the next available slot
  size_t count = 0;                  // Number of elements in the queue

public:
  // Enqueue an event
  bool enqueue(EventData event_data);

  // Dequeue an event
  bool dequeue(EventData &event_data);

  // Check if the queue is empty
  bool empty() const;

  // Check if the queue is full
  bool full() const;

  // Get the current size of the queue
  size_t size() const;
};

#include "EventQueue.h"

bool EventQueue::enqueue(EventData event_data) {
  if (count == capacity) {
    // Queue is full
    return false;
  }
  events[tail] = event_data;
  tail = (tail + 1) % capacity; // Wrap around if necessary
  ++count;
  return true;
}

// Dequeue an event
bool EventQueue::dequeue(EventData &event_data) {
  if (count == 0) {
    // Queue is empty
    return false;
  }
  event_data = events[head];
  head = (head + 1) % capacity; // Wrap around if necessary
  --count;
  return true;
}

// Check if the queue is empty
bool EventQueue::empty() const { return count == 0; }

// Check if the queue is full
bool EventQueue::full() const { return count == capacity; }

// Get the current size of the queue
size_t EventQueue::size() const { return count; }

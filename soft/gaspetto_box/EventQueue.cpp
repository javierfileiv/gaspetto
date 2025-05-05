#include "EventQueue.h"

bool EventQueue::enqueue(Event event) {
  if (count == capacity) {
    // Queue is full
    return false;
  }
  events[tail] = event;
  tail = (tail + 1) % capacity; // Wrap around if necessary
  ++count;
  return true;
}

// Dequeue an event
bool EventQueue::dequeue(Event &event) {
  if (count == 0) {
    // Queue is empty
    return false;
  }
  event = events[head];
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

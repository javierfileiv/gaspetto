#include "EventQueue.h"
#include <stdexcept>          // For std::runtime_error

// Enqueue an event
void EventQueue::enqueue(Event event) {
  std::unique_lock<std::mutex> lock(queueMutex);

  // Check if the queue is full
  if (isFull()) {
    throw std::runtime_error("Queue is full! Cannot enqueue event.");
  }

  // Add the event to the queue
  events[tail] = event;
  tail = (tail + 1) % capacity; // Wrap around if necessary
  count++;

  // Notify waiting threads
  condition.notify_one();
}

// Dequeue an event
Event EventQueue::dequeue() {
  std::unique_lock<std::mutex> lock(queueMutex);

  // Wait until the queue is not empty
  condition.wait(lock, [this]() { return !isEmpty(); });

  // Remove the event from the queue
  Event event = events[head];
  head = (head + 1) % capacity; // Wrap around if necessary
  count--;

  return event;
}

// Check if the queue is empty
bool EventQueue::isEmpty() {
  std::lock_guard<std::mutex> lock(queueMutex);
  return isEmpty();
}

// Get the current size of the queue
size_t EventQueue::size() {
  std::lock_guard<std::mutex> lock(queueMutex);
  return count;
}

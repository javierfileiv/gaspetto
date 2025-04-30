#include "Event.h"
#include <cstddef>            // For size_t
#include <mutex>              // For std::mutex
#include <condition_variable> // For std::condition_variable

class EventQueue {
private:
  static const size_t capacity = 10; // Fixed size of the queue
  Event events[capacity];            // Preallocated array for events
  size_t head = 0;                   // Index of the first element
  size_t tail = 0;                   // Index of the next available slot
  size_t count = 0;                  // Number of elements in the queue
  std::mutex queueMutex;             // Mutex for thread safety
  std::condition_variable condition;

  // Helper function to check if the queue is full
  bool isFull() const { return count == capacity; }

  // Helper function to check if the queue is empty
  bool isEmpty() const { return count == 0; }
  public:
  // Enqueue an event
  void enqueue(Event event);
  // Dequeue an event
  Event dequeue();
  // Check if the queue is empty
  bool isEmpty();
  // Get the current size of the queue
  size_t size();
};
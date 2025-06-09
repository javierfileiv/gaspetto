/*
 * EventQueue.h
 */

#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include <stdint.h>

/*
 * Template class for a generic circular queue.
 * 'T' is a placeholder for any data type (e.g., TelemetryData, int).
 */
template <typename T> class EventQueue {
public:
    /* Constructor: Initializes the queue with a given capacity. */
    explicit EventQueue(uint8_t capacity);

    /* Destructor: Frees dynamically allocated memory. */
    ~EventQueue();

    /* Adds an item to the queue. */
    /* 'const T&' passes the item by reference, avoiding unnecessary copies. */
    bool enqueue(const T &item);

    /* Removes an item from the queue. */
    /* The dequeued item will be copied into the 'item' reference. */
    bool dequeue(T &item);

    /* Checks if the queue is empty. */
    bool IsEmpty() const;

    /* Checks if the queue is full. */
    bool IsFull() const;

    /* Gets the current number of items in the queue. */
    uint8_t GetSize() const;

private:
    T *events; /* Pointer to a dynamically allocated array for items. */
    uint8_t head; /* Index of the front of the queue. */
    uint8_t tail; /* Index of the back of the queue (where next item is inserted). */
    uint8_t count; /* Current number of items in the queue. */
    uint8_t capacity; /* Maximum number of items the queue can hold. */
};

/* --- Template Method Implementations --- */

template <typename T>
EventQueue<T>::EventQueue(uint8_t capacity)
        : head(0)
        , tail(0)
        , count(0)
        , capacity(capacity)
{
    events = new T[capacity]; /* Allocate memory for the event array. */
}

template <typename T> EventQueue<T>::~EventQueue()
{
    delete[] events; /* Free allocated memory when the queue is destroyed. */
}

template <typename T> bool EventQueue<T>::enqueue(const T &item)
{
    if (count == capacity) {
        return false; /* Queue is full. */
    }
    events[tail] = item; /* Copy the item into the queue. */
    tail = (tail + 1) % capacity; /* Move tail, wrap around if needed. */
    ++count;
    return true;
}

template <typename T> bool EventQueue<T>::dequeue(T &item)
{
    if (count == 0) {
        return false; /* Queue is empty. */
    }
    item = events[head]; /* Copy the item out of the queue. */
    head = (head + 1) % capacity; /* Move head, wrap around if needed. */
    --count;
    return true;
}

template <typename T> bool EventQueue<T>::IsEmpty() const
{
    return count == 0;
}

template <typename T> bool EventQueue<T>::IsFull() const
{
    return count == capacity;
}

template <typename T> uint8_t EventQueue<T>::GetSize() const
{
    return count;
}

#endif /* EVENT_QUEUE_H */

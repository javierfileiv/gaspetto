#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include "Event.h"

#ifndef ARDUINO
#include <cstdint>
#endif

#define QUEUE_SIZE 20 /*  Define the size of the queue. */

class EventQueue {
public:
    /*  Enqueue an event. */
    bool enqueue(Event &evt);

    /*  Dequeue an event. */
    bool dequeue(Event &evt);

    /*  Check if the queue is empty. */
    bool IsEmpty() const;

    /*  Check if the queue is full. */
    bool IsFull() const;

    /*  Get the current size of the queue. */
    uint8_t GetSize() const;

private:
    const int capacity = QUEUE_SIZE;
    Event events[QUEUE_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
};
#endif /* EVENT_QUEUE_H */

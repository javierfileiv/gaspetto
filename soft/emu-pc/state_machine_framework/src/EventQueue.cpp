#include "EventQueue.h"

#include "Event.h"

#include <cstdint>

bool EventQueue::enqueue(Event &evt)
{
    if (count == capacity) {
        /*  Queue is full. */
        return false;
    }
    events[tail] = evt;
    tail = static_cast<uint8_t>((tail + 1) % capacity); /*  Wrap around if necessary. */
    ++count;
    return true;
}

/*  Dequeue an event. */
bool EventQueue::dequeue(Event &evt)
{
    if (count == 0) {
        /*  Queue is empty. */
        return false;
    }
    evt = events[head];
    head = static_cast<uint8_t>((head + 1) % capacity); /*  Wrap around if necessary. */
    --count;
    return true;
}

bool EventQueue::IsEmpty() const
{
    return count == 0;
}

bool EventQueue::IsFull() const
{
    return count == capacity;
}

uint8_t EventQueue::GetSize() const
{
    return count;
}

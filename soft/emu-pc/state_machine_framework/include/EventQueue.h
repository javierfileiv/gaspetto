#ifndef EVENT_QUEUE_H
#define EVENT_QUEUE_H

#include "config_event.h"

#ifndef ARDUINO
#include <stdint.h>
#endif

/**
 * @brief Generic event queue for state machine events.
 *
 * @tparam EventT The event type to store.
 * @tparam Capacity Maximum number of events in the queue.
 */
template <typename EventT, uint8_t Capacity = QUEUE_SIZE> class GenericEventQueue {
public:
    /** Enqueue an event. */
    bool enqueue(EventT &evt)
    {
        if (count_ == Capacity) {
            return false;
        }
        events_[tail_] = evt;
        tail_ = (tail_ + 1) % Capacity;
        ++count_;
        return true;
    }

    /** Dequeue an event. */
    bool dequeue(EventT &evt)
    {
        if (count_ == 0) {
            return false;
        }
        evt = events_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        return true;
    }

    /** Check if the queue is empty. */
    bool IsEmpty() const
    {
        return count_ == 0;
    }

    /** Check if the queue is full. */
    bool IsFull() const
    {
        return count_ == Capacity;
    }

    /** Get the current size of the queue. */
    uint8_t GetSize() const
    {
        return count_;
    }

    /** Clear all events from the queue. */
    void clear()
    {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    EventT events_[Capacity];
    uint8_t head_ = 0;
    uint8_t tail_ = 0;
    uint8_t count_ = 0;
};

#endif /* EVENT_QUEUE_H */

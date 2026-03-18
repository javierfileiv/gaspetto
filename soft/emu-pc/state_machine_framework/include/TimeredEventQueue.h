#ifndef TIMERED_EVENT_QUEUE_H
#define TIMERED_EVENT_QUEUE_H

#include "Arduino.h"
#include "Log.h"
#include "config_event.h"

/**
 * @brief Node for storing timed events.
 *
 * @tparam EventT The event type.
 */
template <typename EventT> struct GenericTimedEventNode {
    uint32_t triggerTimeMs;
    EventT event;
    int8_t nextIndex;
    GenericTimedEventNode()
            : triggerTimeMs(0)
            , nextIndex(-1)
    {
    }
};

/**
 * @brief Timed event queue for scheduling delayed events.
 *
 * @tparam EventT The event type.
 * @tparam MaxNodes Maximum number of scheduled events.
 */
template <typename EventT, uint8_t MaxNodes = MAX_TIMED_EVENT_NODES>
class GenericTimeredEventQueue : public Log {
public:
    GenericTimeredEventQueue()
            : headIndex_(-1)
            , freeListHead_(0)
            , lastProcessTime_(millis())
    {
        for (uint8_t i = 0; i < MaxNodes - 1; ++i) {
            eventNodes_[i].nextIndex = i + 1;
        }
        eventNodes_[MaxNodes - 1].nextIndex = -1;
    }

    /** Schedule an event at absolute time. */
    bool scheduleAbsoluteTimeEvent(uint32_t timeMs, EventT evt)
    {
        int8_t newNodeIndex = allocateNode();
        if (newNodeIndex == -1)
            return false;

        eventNodes_[newNodeIndex].triggerTimeMs = timeMs;
        eventNodes_[newNodeIndex].event = evt;
        eventNodes_[newNodeIndex].nextIndex = -1;

        if (headIndex_ == -1 || timeMs < eventNodes_[headIndex_].triggerTimeMs) {
            eventNodes_[newNodeIndex].nextIndex = headIndex_;
            headIndex_ = newNodeIndex;
        } else {
            int current = headIndex_;
            while (eventNodes_[current].nextIndex != -1 &&
                   eventNodes_[current].triggerTimeMs <= timeMs) {
                current = eventNodes_[current].nextIndex;
            }
            eventNodes_[newNodeIndex].nextIndex = eventNodes_[current].nextIndex;
            eventNodes_[current].nextIndex = newNodeIndex;
        }
        return true;
    }

    /** Schedule an event after a delay. */
    bool scheduleEventDelayed(uint32_t delayMs, EventT event)
    {
        return scheduleAbsoluteTimeEvent(millis() + delayMs, event);
    }

    /** Process pending events and post them to the active object. */
    template <typename ActiveObjectT> void process(ActiveObjectT &ao)
    {
        uint32_t currentTime = millis();
        uint32_t elapsedTime = currentTime - lastProcessTime_;
        int8_t current = headIndex_;

        while (current != -1) {
            if (eventNodes_[current].triggerTimeMs > elapsedTime)
                eventNodes_[current].triggerTimeMs -= elapsedTime;
            else
                eventNodes_[current].triggerTimeMs = 0;
            current = eventNodes_[current].nextIndex;
        }

        while (headIndex_ != -1 && eventNodes_[headIndex_].triggerTimeMs == 0) {
            uint8_t temp = headIndex_;
            ao.postEvent(eventNodes_[temp].event);
            headIndex_ = eventNodes_[headIndex_].nextIndex;
            freeNode(temp);
        }
        lastProcessTime_ = currentTime;
    }

    /** Clear all scheduled events. */
    void clear()
    {
        headIndex_ = -1;
        freeListHead_ = 0;
        for (uint8_t i = 0; i < MaxNodes - 1; ++i)
            eventNodes_[i].nextIndex = i + 1;
        eventNodes_[MaxNodes - 1].nextIndex = -1;
    }

private:
    int8_t allocateNode()
    {
        int8_t allocatedIndex = freeListHead_;
        if (allocatedIndex == -1)
            return -1;
        freeListHead_ = eventNodes_[allocatedIndex].nextIndex;
        return allocatedIndex;
    }

    void freeNode(uint8_t index)
    {
        eventNodes_[index].nextIndex = freeListHead_;
        freeListHead_ = index;
    }

    uint32_t lastProcessTime_;
    GenericTimedEventNode<EventT> eventNodes_[MaxNodes];
    int8_t headIndex_;
    int8_t freeListHead_;
};

#endif /* TIMERED_EVENT_QUEUE_H */

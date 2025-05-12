#include "TimeredEventQueue.h"

#include "Arduino.h"

TimeredEventQueue::TimeredEventQueue()
        : headIndex_(-1)
        , freeListHead_(0)
        , lastProcessTime_(millis())
{
    /*  Initialize the free list. */
    for (uint8_t i = 0; i < MAX_TIMED_EVENT_NODES - 1; ++i) {
        eventNodes_[i].nextIndex = i + 1;
    }
    eventNodes_[MAX_TIMED_EVENT_NODES - 1].nextIndex = -1;
}

bool TimeredEventQueue::scheduleAbsoluteTimeEvent(uint32_t timeMs, Event evt)
{
    uint8_t newNodeIndex = allocateNode();
    if (newNodeIndex == -1)
        return false; /*  Queue is full. */
#ifdef DEBUG_GASPETTO
    Serial.print("scheduleEvent(): ");
    Serial.print(reinterpret_cast<const char *>(
            event_to_string[static_cast<int>(evt.getEventId())].str));
    Serial.print(", ");
    Serial.print("Command: ");
    Serial.print(reinterpret_cast<const char *>(
            command_to_string[static_cast<int>(evt.getCommand())].str));
    Serial.print(", ");
    Serial.print("Trigger Time: ");
    Serial.print(timeMs);
    Serial.print(" ms, ");
    Serial.print("Current Time: ");
    Serial.print(millis());
    Serial.println(" ms");
#endif
    eventNodes_[newNodeIndex].triggerTimeMs = timeMs;
    eventNodes_[newNodeIndex].event = evt;
    eventNodes_[newNodeIndex].nextIndex = -1;
    if (headIndex_ == -1 || timeMs < eventNodes_[headIndex_].triggerTimeMs) {
        eventNodes_[newNodeIndex].nextIndex = headIndex_;
        headIndex_ = newNodeIndex;
    } else {
        int current = headIndex_;
        while (eventNodes_[current].nextIndex != -1 &&
               eventNodes_[eventNodes_[current].nextIndex].triggerTimeMs <= timeMs) {
            current = eventNodes_[current].nextIndex;
        }
        eventNodes_[newNodeIndex].nextIndex = eventNodes_[current].nextIndex;
        eventNodes_[current].nextIndex = newNodeIndex;
    }
    /*  Update the last process time. */
    return true;
}

bool TimeredEventQueue::scheduleEventDelayed(uint32_t delayMs, Event event)
{
    return scheduleAbsoluteTimeEvent(millis() + delayMs, event);
}

void TimeredEventQueue::processEvents(ActiveObject &ao)
{
    uint32_t currentTime = millis();
    uint32_t elapsedTime = currentTime - lastProcessTime_;
    int8_t current = headIndex_;

    /* Adjust elapsed time between calls. */
    while (current != -1) {
        if (eventNodes_[current].triggerTimeMs > elapsedTime)
            eventNodes_[current].triggerTimeMs -= elapsedTime;
        else
            eventNodes_[current].triggerTimeMs = 0; // Event is due
        current = eventNodes_[current].nextIndex;
    }
    while (headIndex_ != -1 && eventNodes_[headIndex_].triggerTimeMs == 0) {
        uint8_t temp = headIndex_;

        ao.postEvent(eventNodes_[temp].event);
        headIndex_ = eventNodes_[headIndex_].nextIndex;
        freeNode(temp);
    }
    /*  Update the last process time. */
    lastProcessTime_ = currentTime;
}

void TimeredEventQueue::clear(void)
{
    headIndex_ = -1;
    freeListHead_ = 0;
    for (uint8_t i = 0; i < MAX_TIMED_EVENT_NODES - 1; ++i)
        eventNodes_[i].nextIndex = i + 1;
    eventNodes_[MAX_TIMED_EVENT_NODES - 1].nextIndex = -1;
}

uint8_t TimeredEventQueue::allocateNode(void)
{
    uint8_t allocatedIndex = freeListHead_;

    if (allocatedIndex == -1)
        return -1; /*  No free nodes. */
    freeListHead_ = eventNodes_[allocatedIndex].nextIndex;
    return allocatedIndex;
}

void TimeredEventQueue::freeNode(uint8_t index)
{
    eventNodes_[index].nextIndex = freeListHead_;
    freeListHead_ = index;
}

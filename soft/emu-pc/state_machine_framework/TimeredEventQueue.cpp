#include "TimeredEventQueue.h"

#include "Arduino.h"

TimeredEventQueue::TimeredEventQueue()
        : headIndex_(-1)
        , freeListHead_(0)
{
    /*  Initialize the free list. */
    for (uint8_t i = 0; i < MAX_TIMED_EVENT_NODES - 1; ++i) {
        eventNodes_[i].nextIndex = i + 1;
    }
    eventNodes_[MAX_TIMED_EVENT_NODES - 1].nextIndex = -1;
}

bool TimeredEventQueue::scheduleEvent(uint32_t timeMs, Event event)
{
    uint8_t newNodeIndex = allocateNode();
    if (newNodeIndex == -1) {
        return false; /*  Queue is full. */
    }

    eventNodes_[newNodeIndex].triggerTimeMs = timeMs;
    eventNodes_[newNodeIndex].event = event;
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
    return true;
}

bool TimeredEventQueue::scheduleEventDelayed(uint32_t delayMs, Event event)
{
    return scheduleEvent(millis() + delayMs, event);
}

void TimeredEventQueue::processEvents(ActiveObject &ao)
{
    uint32_t currentTime = millis();
    while (headIndex_ != -1 && eventNodes_[headIndex_].triggerTimeMs <= currentTime) {
        Serial.print("Processing event: ");
        Serial.print("Event: ");
        Serial.print(reinterpret_cast<const char *>(
                event_to_string[static_cast<int>(eventNodes_[headIndex_].event.getEventId())].str));
        Serial.print(", ");
        Serial.print("Command: ");
        Serial.print(reinterpret_cast<const char *>(
                command_to_string[static_cast<int>(eventNodes_[headIndex_].event.getCommand())]
                        .str));
        Serial.print(", ");
        Serial.print("Trigger Time: ");
        Serial.print(eventNodes_[headIndex_].triggerTimeMs);
        Serial.print(" ms, ");
        Serial.print("Current Time: ");
        Serial.print(currentTime);
        Serial.println(" ms");
        ao.postEvent(eventNodes_[headIndex_].event);
        uint8_t temp = headIndex_;
        headIndex_ = eventNodes_[headIndex_].nextIndex;
        freeNode(temp);
    }
}

void TimeredEventQueue::clear(void)
{
    headIndex_ = -1;
    freeListHead_ = 0;
    for (uint8_t i = 0; i < MAX_TIMED_EVENT_NODES - 1; ++i) {
        eventNodes_[i].nextIndex = i + 1;
    }
    eventNodes_[MAX_TIMED_EVENT_NODES - 1].nextIndex = -1;
}

uint8_t TimeredEventQueue::allocateNode(void)
{
    if (freeListHead_ == -1) {
        return -1; /*  No free nodes. */
    }
    uint8_t allocatedIndex = freeListHead_;
    freeListHead_ = eventNodes_[freeListHead_].nextIndex;
    return allocatedIndex;
}

void TimeredEventQueue::freeNode(uint8_t index)
{
    eventNodes_[index].nextIndex = freeListHead_;
    freeListHead_ = index;
}

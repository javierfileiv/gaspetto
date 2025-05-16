#ifndef TIMERED_EVENT_QUEUE_H
#define TIMERED_EVENT_QUEUE_H

#include "ActiveObject.h"
#include "Arduino.h"
#include "Event.h" /*  Assuming you have your ActiveObject definition. */

#define MAX_TIMED_EVENT_NODES 15 /*  Define a maximum number of scheduled events. */

struct TimedEventNode {
    uint32_t triggerTimeMs;
    Event event;
    int8_t nextIndex; /*  Index of the next node in the array (-1 for null). */
    TimedEventNode()
            : triggerTimeMs(0)
            , nextIndex(-1)
    {
    }
};

class TimeredEventQueue {
public:
    TimeredEventQueue();
    bool scheduleAbsoluteTimeEvent(uint32_t timeMs, Event event); /*  Schedule based on absolute
                                                                     time. */
    bool scheduleEventDelayed(uint32_t delayMs, Event event); /*  Schedule based on delay. */
    void processEvents(ActiveObject &ao);
    void clear(void); /*  Reset the queue. */

private:
    uint32_t lastProcessTime_;
    TimedEventNode eventNodes_[MAX_TIMED_EVENT_NODES];
    int8_t headIndex_; /*  Index of the first event in the sorted list (-1 if
                          empty). */
    int8_t freeListHead_; /*  Index of the first free node in the array (-1 if
                             full). */
    uint8_t allocateNode(void);
    void freeNode(uint8_t index);
};

#endif /* TIMERED_EVENT_QUEUE_H */

#ifndef TIMERED_EVENT_QUEUE_H
#define TIMERED_EVENT_QUEUE_H

#include "ActiveObject.h"
#include "Arduino.h"
#include "Event.h"
#include "Log.h"
#include "config_event.h"

struct TimedEventNode {
    uint32_t triggerTimeMs;
    Event event;
    int8_t nextIndex;
    TimedEventNode()
            : triggerTimeMs(0)
            , nextIndex(-1)
    {
    }
};

class TimeredEventQueue : public Log {
public:
    TimeredEventQueue();
    bool scheduleAbsoluteTimeEvent(uint32_t timeMs, const Event &event); /*  Schedule on absolute
                                                                            time. */
    bool scheduleEventDelayed(uint32_t delayMs, const Event &event); /*  Schedule on delay. */
    void processEvents(ActiveObject &ao);
    void clear();

    static const char *eventIdToString(EventId id)
    {
        switch (id) {
        case EventId::NONE:
            return "NONE";
        case EventId::TIMER_ELAPSED:
            return "TIMER_ELAPSED";
        case EventId::ACTION:
            return "ACTION";
        case EventId::BUTTON_PRESSED:
            return "BUTTON_PRESSED";

        case EventId::MAX_EVENT_ID:
            return "MAX_EVENT_ID";
        default:
            return "UNKNOWN_EVENT_ID";
        }
    }

    static const char *commandIdToString(CommandId id)
    {
        switch (id) {
        case CommandId::NONE:
            return "NONE";
        case CommandId::MOTOR_FORWARD:
            return "MOTOR_FORWARD";
        case CommandId::MOTOR_BACKWARD:
            return "MOTOR_BACKWARD";
        case CommandId::MOTOR_RIGHT:
            return "MOTOR_RIGHT";
        case CommandId::MOTOR_LEFT:
            return "MOTOR_LEFT";
        case CommandId::MOTOR_STOP:
            return "MOTOR_STOP";
        case CommandId::MAX_COMMAND_ID:
            return "MAX_COMMAND_ID";
        default:
            return "UNKNOWN_COMMAND_ID";
        }
    }

private:
    uint32_t lastProcessTime_;
    TimedEventNode eventNodes_[MAX_TIMED_EVENT_NODES];
    int8_t headIndex_;
    int8_t freeListHead_;
    int8_t allocateNode();
    void freeNode(uint8_t index);
};

#endif /* TIMERED_EVENT_QUEUE_H */

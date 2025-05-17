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

    /*  Stringify EventId for debugging. */
    static const char *eventIdToString(EventId id)
    {
        switch (id) {
        case EventId::NONE:
            return "NONE";
        case EventId::TIMER_ELAPSED:
            return "TIMER_ELAPSED";
        case EventId::NRF_IRQ:
            return "NRF_IRQ";
        case EventId::BUTTON_PRESSED:
            return "BUTTON_PRESSED";
        case EventId::MAX_EVENT_ID:
            return "MAX_EVENT_ID";
        default:
            return "UNKNOWN_EVENT_ID";
        }
    }

    /*  Stringify CommandId for debugging. */
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
    const int capacity = QUEUE_SIZE;
    Event events[QUEUE_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
};
#endif /* EVENT_QUEUE_H */

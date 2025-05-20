#ifndef MOTOR_EVENT_H
#define MOTOR_EVENT_H

#include "Event.h"

#ifndef ARDUINO
#else
#include "Arduino.h"
#endif

class MotorEvent : public Event {
public:
    MotorEvent()
            : Event(EventId::NONE, CommandId::NONE)
    {
    }
    MotorEvent(EventId eventId, CommandId commandId)
            : Event(eventId, commandId)
    {
    }
};

#endif /* MOTOR_EVENT_H */

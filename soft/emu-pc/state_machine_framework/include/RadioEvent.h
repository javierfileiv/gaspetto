#ifndef RADIO_EVENT_H
#define RADIO_EVENT_H

#ifndef ARDUINO
#else
#include "Arduino.h"
#endif

#include "Event.h"

class RadioEvent : public Event {
public:
    RadioEvent()
            : Event(EventId::NONE, CommandId::NONE)
    {
    }
    RadioEvent(EventId eventId, CommandId commandId)
            : Event(eventId, commandId)
    {
    }
};

#endif /* RADIO_EVENT_H */

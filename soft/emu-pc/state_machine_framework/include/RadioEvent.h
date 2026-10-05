#pragma once

#ifndef ARDUINO
#else
#include "Arduino.h"
#endif

#include "GenericEvent.h"

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

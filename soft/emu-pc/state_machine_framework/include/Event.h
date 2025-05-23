#ifndef EVENT_H
#define EVENT_H

#ifndef ARDUINO
#include <cstdint>
#else
#include "Arduino.h"
#endif

enum class EventId : uint8_t {
    NONE,
    TIMER_ELAPSED,
    ACTION,
    BUTTON_PRESSED,
    RADIO_TX,
    MAX_EVENT_ID
};
enum class CommandId : uint8_t {
    NONE,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_RIGHT,
    MOTOR_LEFT,
    MOTOR_STOP,
    MAX_COMMAND_ID
};

// Stringify helpers for EventId and CommandId
static inline const char *eventIdToString(EventId id)
{
    switch (id) {
    case EventId::NONE:
        return "NONE";
    case EventId::TIMER_ELAPSED:
        return "TIMER_ELAPSED";
    case EventId::ACTION:
        return "NRF_IRQ";
    case EventId::BUTTON_PRESSED:
        return "BUTTON_PRESSED";
    case EventId::MAX_EVENT_ID:
        return "MAX_EVENT_ID";
    default:
        return "UNKNOWN_EVENT_ID";
    }
}

static inline const char *commandIdToString(CommandId id)
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

struct __attribute__((packed)) EventPacket {
    uint8_t eventId;
    uint8_t commandId;
};

class Event {
public:
    Event(EventId eventId = EventId::NONE, CommandId command = CommandId::NONE)
            : eventId(eventId)
            , command(command)
    {
    }
    virtual ~Event() = default;
    EventId getEventId() const
    {
        return eventId;
    }
    CommandId getCommand() const
    {
        return command;
    }
    void setEventId(EventId id)
    {
        eventId = id;
    }
    void setCommand(CommandId cmd)
    {
        command = cmd;
    }
    void toPacket(EventPacket &packet) const
    {
        packet.eventId = static_cast<uint8_t>(eventId);
        packet.commandId = static_cast<uint8_t>(command);
    }
    static Event fromPacket(const EventPacket &packet)
    {
        return Event(static_cast<EventId>(packet.eventId),
                     static_cast<CommandId>(packet.commandId));
    }
    static constexpr uint8_t packetSize()
    {
        return sizeof(EventPacket);
    }

protected:
    EventId eventId;
    CommandId command;
};

#endif /* EVENT_H */

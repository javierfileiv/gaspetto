#pragma once
#include <cstdint>

enum class EventId {
  TIMER_ELAPSED,
  NRF_IRQ,
  BUTTON_PRESSED,
  NONE,
  MAX_EVENT_ID
};

enum class StateId { IDLE, PROCESSING, PAUSED, MAX_STATE_ID };

enum class CommandId {
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_RIGHT,
  MOTOR_LEFT,
  MOTOR_STOP,
  NONE,
  MAX_COMMAND_ID
};

const struct {
  CommandId command;
  const uint8_t *str;
} command_to_string[static_cast<std::size_t>(CommandId::MAX_COMMAND_ID)] = {
    {CommandId::MOTOR_FORWARD,
     reinterpret_cast<const uint8_t *>("MOTOR_FORWARD")},
    {CommandId::MOTOR_BACKWARD,
     reinterpret_cast<const uint8_t *>("MOTOR_BACKWARD")},
    {CommandId::MOTOR_RIGHT, reinterpret_cast<const uint8_t *>("MOTOR_RIGHT")},
    {CommandId::MOTOR_LEFT, reinterpret_cast<const uint8_t *>("MOTOR_LEFT")},
    {CommandId::MOTOR_STOP, reinterpret_cast<const uint8_t *>("MOTOR_STOP")},
    {CommandId::NONE, reinterpret_cast<const uint8_t *>("NONE")},

};

const struct {
  EventId event;
  const uint8_t *str;
} event_to_string[static_cast<std::size_t>(EventId::MAX_EVENT_ID)] = {
    {EventId::TIMER_ELAPSED,
     reinterpret_cast<const uint8_t *>("TIMER_ELAPSED")},
    {EventId::NRF_IRQ, reinterpret_cast<const uint8_t *>("NRF_IRQ")},
    {EventId::BUTTON_PRESSED,
     reinterpret_cast<const uint8_t *>("BUTTON_PRESSED")},
    {EventId::NONE, reinterpret_cast<const uint8_t *>("NONE")},
};

class Event {
public:
  Event() : event(EventId::NONE), command(CommandId::MOTOR_STOP) {}
  Event(EventId eventId, CommandId commandId)
      : event(eventId), command(commandId) {}
  void setEventId(EventId eventId) { event = eventId; }
  void setCommand(CommandId commandId) { command = commandId; }
  EventId getEventId() const { return event; }
  CommandId getCommand() const { return command; }

private:
  EventId event;
  CommandId command;
};

#pragma once
#include <cstdint>

enum class EventId { TIMER_ELAPSED, NRF_IRQ, NONE };

enum class StateId { IDLE, PROCESSING, PAUSED, MAX_STATE_ID };

enum class CommandId {
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_RIGHT,
  MOTOR_LEFT,
  MOTOR_STOP,
  MAX_COMMAND_ID
};

const struct {
  CommandId command;
  uint8_t str[20];
} command_to_string[] = {
    {CommandId::MOTOR_FORWARD, "MOTOR_FORWARD"},
    {CommandId::MOTOR_BACKWARD, "MOTOR_BACKWARD"},
    {CommandId::MOTOR_RIGHT, "MOTOR_RIGHT"},
    {CommandId::MOTOR_LEFT, "MOTOR_LEFT"},
    {CommandId::MOTOR_STOP, "MOTOR_STOP"},
};

const struct {
  EventId event;
  uint8_t str[20];
} event_to_string[] = {
    {EventId::TIMER_ELAPSED, "TIMER_ELAPSED"},
    {EventId::NRF_IRQ, "NRF_IRQ"},
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

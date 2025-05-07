#pragma once

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

class Event{
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

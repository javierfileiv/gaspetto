#ifndef CAR_EVENTS_H
#define CAR_EVENTS_H

#include "Event.h"

#ifndef ARDUINO
#include <cstdint>
#else
#include "Arduino.h"
#endif

/**
 * @brief Event type identifiers for GaspettoCar.
 */
enum class EventId : uint8_t {
    NONE, /**< No event. */
    TIMER_ELAPSED, /**< Timer expiration. */
    ACTION, /**< Motor command. */
    BUTTON_PRESSED, /**< Button press. */
    RADIO_TX, /**< Radio transmission. */
};

/**
 * @brief Command identifiers for ACTION events.
 */
enum class CommandId : uint8_t {
    NONE, /**< No command. */
    MOTOR_FORWARD, /**< Drive forward. */
    MOTOR_BACKWARD, /**< Drive backward. */
    MOTOR_TURN_RIGHT, /**< Turn right. */
    MOTOR_TURN_LEFT, /**< Turn left. */
    MOTOR_STOP, /**< Stop motors. */
    MAX_COMMAND_ID /**< Sentinel. */
};

/**
 * @brief Application-specific Event type for GaspettoCar.
 */
using Event = GenericEvent<EventId, CommandId>;

/**
 * @brief Convert EventId to string for debugging.
 */
inline const char *eventIdToString(EventId id)
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
    case EventId::RADIO_TX:
        return "RADIO_TX";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief Convert CommandId to string for debugging.
 */
inline const char *commandIdToString(CommandId id)
{
    switch (id) {
    case CommandId::NONE:
        return "NONE";
    case CommandId::MOTOR_FORWARD:
        return "MOTOR_FORWARD";
    case CommandId::MOTOR_BACKWARD:
        return "MOTOR_BACKWARD";
    case CommandId::MOTOR_TURN_RIGHT:
        return "MOTOR_TURN_RIGHT";
    case CommandId::MOTOR_TURN_LEFT:
        return "MOTOR_TURN_LEFT";
    case CommandId::MOTOR_STOP:
        return "MOTOR_STOP";
    default:
        return "UNKNOWN";
    }
}

#endif /* CAR_EVENTS_H */

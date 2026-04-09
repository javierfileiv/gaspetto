#pragma once

#include <stdint.h>

/* Wire protocol: these values are transmitted over RF24 and must remain stable
 * across every sender/receiver that exchanges motor commands. */
enum class CommandId : uint8_t {
    NONE = 0,
    MOTOR_FORWARD = 1,
    MOTOR_BACKWARD = 2,
    MOTOR_TURN_RIGHT = 3,
    MOTOR_TURN_LEFT = 4,
    MOTOR_STOP = 5,
    MAX_COMMAND_ID = 6,
};

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

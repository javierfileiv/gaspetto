#include "IdleState.h"

#include "GaspettoCar.h"
#include "__assert.h"

#include <cstdint>

const bool FORWARD = true;
const bool BACKWARD = false;

void IdleState::enter()
{
    active_object->enterLowPowerMode();
}

void IdleState::processEvent(Event &evt)
{
    GaspettoCar *ao = static_cast<GaspettoCar *>(active_object);

    switch (evt.getEventId()) {
    case EventId::ACTION: {
        switch (evt.getCommand()) {
        case CommandId::MOTOR_FORWARD:
            ao->setMotor(FORWARD, MOTOR_SPEED, DISTANCE_CM_FWD_BWD, FORWARD, MOTOR_SPEED,
                         DISTANCE_CM_FWD_BWD);
            active_object->transitionTo(StateId::PROCESSING);
            logln("Transition to PROCESSING (Turn Forward)");
            break;
        case CommandId::MOTOR_BACKWARD:
            ao->setMotor(BACKWARD, MOTOR_SPEED, DISTANCE_CM_FWD_BWD, BACKWARD, MOTOR_SPEED, 20);
            active_object->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Turn Backward)"));
            break;
        case CommandId::MOTOR_LEFT:
            ao->setMotor(BACKWARD, MOTOR_SPEED, DISTANCE_CM_TURN_LEFT, FORWARD, MOTOR_SPEED,
                         DISTANCE_CM_FWD_BWD);
            active_object->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Turn Left)"));
            break;
        case CommandId::MOTOR_RIGHT:
            ao->setMotor(FORWARD, MOTOR_SPEED, DISTANCE_CM_FWD_BWD, BACKWARD, MOTOR_SPEED,
                         DISTANCE_CM_TURN_RIGHT);

            active_object->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Turn Right)"));
            break;

        case CommandId::MOTOR_STOP:
            ao->stopMotorLeft();
            ao->stopMotorRight();
            logln(F("IdleState: Received MOTOR_STOP while already idle. Ensuring motors are off."));
            ao->enterLowPowerMode();
            break;
        default:
            log(F("IdleState: Unhandled ACTION command: "));
            logln(Event::commandIdToString(evt.getCommand()));
            break;
        }
        break;
    default:
        log(F("IdleState: Unhandled event ID: "));
        logln(Event::eventIdToString(evt.getEventId()));
        assert(false); // Unhandled event ID
        break;
    }
    }
}

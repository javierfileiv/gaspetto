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
    GaspettoCar *car = static_cast<GaspettoCar *>(active_object);

    switch (evt.getEventId()) {
    case EventId::ACTION: {
        switch (evt.getCommand()) {
        case CommandId::MOTOR_FORWARD:
            car->setMotor(FORWARD, INITIAL_MOTOR_SPEED, FORWARD, INITIAL_MOTOR_SPEED);
            active_object->transitionTo(StateId::PROCESSING);
            logln("Transition to PROCESSING (Move Forward)");
            break;
        case CommandId::MOTOR_BACKWARD:
            car->setMotor(BACKWARD, INITIAL_MOTOR_SPEED, BACKWARD, INITIAL_MOTOR_SPEED);
            active_object->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Move Backward)"));
            break;
        case CommandId::MOTOR_LEFT:
            car->setMotor(BACKWARD, TURN_MOTOR_SPEED, FORWARD, INITIAL_MOTOR_SPEED);
            active_object->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Turn Left)"));
            break;
        case CommandId::MOTOR_RIGHT:
            car->setMotor(FORWARD, INITIAL_MOTOR_SPEED, BACKWARD, TURN_MOTOR_SPEED);

            active_object->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Turn Right)"));
            break;

        case CommandId::MOTOR_STOP:
            car->stopMotorLeft();
            car->stopMotorRight();
            logln(F("IdleState: Received MOTOR_STOP while already idle. Ensuring motors are off."));
            car->enterLowPowerMode();
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
        assert(false);
        break;
    }
    }
}

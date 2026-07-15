#include "IdleState.h"

#include "GCar.h"
#include "__assert.h"

#include <cstdint>

void IdleState::enter()
{
    active_object_->enterLowPowerMode();
}

void IdleState::processEvent(Event &evt)
{
    GCar *car = static_cast<GCar *>(active_object_);

    switch (evt.getEventId()) {
    case EventId::ACTION: {
        switch (evt.getPayload()) {
        case CommandId::MOTOR_FORWARD:
            car->setMotorStraightDrive(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
            car->transitionTo(StateId::PROCESSING);
            logln("Transition to PROCESSING (Move Forward)");
            break;
        case CommandId::MOTOR_BACKWARD:
            car->setMotorStraightDrive(-INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
            car->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Move Backward)"));
            break;
        case CommandId::MOTOR_TURN_LEFT:
            car->setMotorTurnInPlace(-90.0f, TURN_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
            car->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Turn Left)"));
            break;
        case CommandId::MOTOR_TURN_RIGHT:
            car->setMotorTurnInPlace(90.0f, TURN_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
            car->transitionTo(StateId::PROCESSING);
            logln(F("Transition to PROCESSING (Turn Right)"));
            break;
        case CommandId::QUEUE_CLEAR:
            car->clearEventQueue();
            logln(F("IdleState: Queue clear command received. Clearing queue."));
            /* Fall through to default to re-enter sleep state. */
        default:
            /* Re-set state to enter low power. */
            car->stopBothMotors();
            log(F("IdleState: Re enter sleep state. Stop or unhandled ACTION command: "));
            logln(commandIdToString(evt.getPayload()));
            car->transitionTo(StateId::IDLE);
            break;
        }
        break;
    default:
        log(F("IdleState: Unhandled event ID: "));
        logln(eventIdToString(evt.getEventId()));
#ifdef ARDUINO
        while (1)
            ;
#endif
        break;
    }
    }
}

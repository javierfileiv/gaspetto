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
            LOGLN("Transition to PROCESSING (Move Forward)");
            break;
        case CommandId::MOTOR_BACKWARD:
            car->setMotorStraightDrive(-INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
            car->transitionTo(StateId::PROCESSING);
            LOGLN(F("Transition to PROCESSING (Move Backward)"));
            break;
        /* A quarter turn needs more time than a forward burst, and this
         * duration is only the safety net: the turn stops on its angle, which
         * the controller's stability check catches well before it */
        case CommandId::MOTOR_TURN_LEFT:
            /* Yaw grows counter-clockwise, so a left turn adds 90 degrees to
             * whatever heading the car has now */
            car->setMotorTurnBy(90.0f, TURN_MOTOR_SPEED, TURN_MOTOR_TIMEOUT_MS);
            car->transitionTo(StateId::PROCESSING);
            LOGLN(F("Transition to PROCESSING (Turn Left)"));
            break;
        case CommandId::MOTOR_TURN_RIGHT:
            car->setMotorTurnBy(-90.0f, TURN_MOTOR_SPEED, TURN_MOTOR_TIMEOUT_MS);
            car->transitionTo(StateId::PROCESSING);
            LOGLN(F("Transition to PROCESSING (Turn Right)"));
            break;
        case CommandId::QUEUE_CLEAR:
            car->clearEventQueue();
            LOGLN(F("IdleState: Queue clear command received. Clearing queue."));
            /* Fall through to default to re-enter sleep state. */
        default:
            /* Re-set state to enter low power. */
            car->stopBothMotors();
            LOG(F("IdleState: Re enter sleep state. Stop or unhandled ACTION command: "));
            LOGLN(commandIdToString(evt.getPayload()));
            car->transitionTo(StateId::IDLE);
            break;
        }
        break;
    default:
        LOG(F("IdleState: Unhandled event ID: "));
        LOGLN(eventIdToString(evt.getEventId()));
#ifdef ARDUINO
        while (1)
            ;
#endif
        break;
    }
    }
}

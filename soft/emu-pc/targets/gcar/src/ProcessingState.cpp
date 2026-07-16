#include "ProcessingState.h"

#include "GCar.h"

#include <Arduino.h>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

void ProcessingState::processEvent(Event &evt)
{
    LOG(F("ProcessingState: Processing EventId: "));
    LOG(eventIdToString(evt.getEventId()));
    LOG(F(", CommandId: "));
    LOG(commandIdToString(evt.getPayload()));
    LOGLN(F("."));

    GCar *ao = static_cast<GCar *>(active_object_);

    switch (evt.getEventId()) {
    case EventId::ACTION: {
        switch (evt.getPayload()) {
        case CommandId::MOTOR_STOP:
            ao->stopBothMotors();
            LOGLN(F("ProcessingState: -> Transition to IDLE (Explicit Stop Command)"));
            ao->transitionTo(StateId::IDLE);
            break;
        case CommandId::QUEUE_CLEAR:
            ao->stopBothMotors();
            ao->clearEventQueue();
            LOGLN(F("ProcessingState: -> Transition to IDLE (Queue Clear Command)"));
            ao->transitionTo(StateId::IDLE);
            break;
        case CommandId::MOTOR_FORWARD:
        case CommandId::MOTOR_BACKWARD:
        case CommandId::MOTOR_TURN_LEFT:
        case CommandId::MOTOR_TURN_RIGHT:
            LOGLN(F("ProcessingState: Ignoring new motor command while already processing a movement."));
            break;
        default:
            LOG(F("ProcessingState: Unhandled ACTION command: "));
            LOGLN(commandIdToString(evt.getPayload()));
            break;
        }
    } break;
    default: {
        LOG(F("ProcessingState: Unhandled EventId: "));
        LOGLN(eventIdToString(evt.getEventId()));
        break;
    }
    }
}

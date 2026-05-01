#include "ProcessingState.h"

#include "GaspettoCar.h"

#include <Arduino.h>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

void ProcessingState::processEvent(Event &evt)
{
    log(F("ProcessingState: Processing EventId: "));
    log(eventIdToString(evt.getEventId()));
    log(F(", CommandId: "));
    log(commandIdToString(evt.getPayload()));
    logln(F("."));

    GaspettoCar *ao = static_cast<GaspettoCar *>(active_object_);

    switch (evt.getEventId()) {
    case EventId::ACTION: {
        switch (evt.getPayload()) {
        case CommandId::MOTOR_STOP:
            ao->stopBothMotors();
            logln(F("ProcessingState: -> Transition to IDLE (Explicit Stop Command)"));
            ao->transitionTo(StateId::IDLE);
            break;
        case CommandId::QUEUE_CLEAR:
            ao->stopBothMotors();
            ao->clearEventQueue();
            logln(F("ProcessingState: -> Transition to IDLE (Queue Clear Command)"));
            ao->transitionTo(StateId::IDLE);
            break;
        case CommandId::MOTOR_FORWARD:
        case CommandId::MOTOR_BACKWARD:
        case CommandId::MOTOR_TURN_LEFT:
        case CommandId::MOTOR_TURN_RIGHT:
            logln(F("ProcessingState: Ignoring new motor command while already processing a movement."));
            break;
        default:
            log(F("ProcessingState: Unhandled ACTION command: "));
            logln(commandIdToString(evt.getPayload()));
            break;
        }
    } break;
    default: {
        log(F("ProcessingState: Unhandled EventId: "));
        logln(eventIdToString(evt.getEventId()));
        break;
    }
    }
}

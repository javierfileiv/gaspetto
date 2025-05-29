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
    log(Event::eventIdToString(evt.getEventId()));
    log(F(", CommandId: "));
    log(Event::commandIdToString(evt.getCommand()));
    logln(F("."));

    GaspettoCar *ao = static_cast<GaspettoCar *>(active_object);

    switch (evt.getEventId()) {
    case EventId::ACTION: {
        switch (evt.getCommand()) {
        case CommandId::MOTOR_STOP:
            ao->stopMotorLeft();
            ao->stopMotorRight();
            ao->transitionTo(StateId::IDLE);
            logln(F("ProcessingState: -> Transition to IDLE (Explicit Stop Command)"));
            break;
        case CommandId::MOTOR_FORWARD:
        case CommandId::MOTOR_BACKWARD:
        case CommandId::MOTOR_LEFT:
        case CommandId::MOTOR_RIGHT:
            logln(F("ProcessingState: Ignoring new motor command while already processing a movement."));
            break;
        default:
            log(F("ProcessingState: Unhandled ACTION command: "));
            logln(Event::commandIdToString(evt.getCommand()));
            break;
        }
    } break;
    default: {
        log(F("ProcessingState: Unhandled EventId: "));
        logln(Event::eventIdToString(evt.getEventId()));
        break;
    }
    }
}

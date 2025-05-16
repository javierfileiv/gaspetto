#include "ProcessingState.h"

#include "GaspettoCar.h"

#include <Arduino.h>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

#define DUTY_CYCLE 30

void ProcessingState::processEvent(Event evt)
{
    switch (evt.getCommand()) {
    case CommandId::MOTOR_FORWARD:
        /* Moving forward */
        Serial.println("Moving forward...\n");
        static_cast<GaspettoCar *>(state_machine)->SetMotor(FORWARD, 5, FORWARD, 5);
        break;
    case CommandId::MOTOR_BACKWARD:
        /* Moving backward */
        Serial.println("Moving backward...\n");
        static_cast<GaspettoCar *>(state_machine)->SetMotor(BACKWARD, 5, BACKWARD, 5);
        break;
    case CommandId::MOTOR_RIGHT:
        /* Turning right */
        Serial.println("Turning right...\n");
        static_cast<GaspettoCar *>(state_machine)->SetMotor(FORWARD, 5, BACKWARD, 5);
        break;
    case CommandId::MOTOR_LEFT:
        /* Turning left */
        Serial.println("Turning left...\n");
        static_cast<GaspettoCar *>(state_machine)->SetMotor(BACKWARD, 5, FORWARD, 5);
        break;
    case CommandId::MOTOR_STOP:
        /* Stopping and transitioning to IDLE state */
        Serial.println("Stopping...\n");
        static_cast<GaspettoCar *>(state_machine)->stopMotor();
        static_cast<GaspettoCar *>(state_machine)->transitionTo(StateId::IDLE);
        break;
    default:
        /* Unknown event in PROCESSING state */
        Serial.println("Unknown event in PROCESSING state.\n");
        assert(true);
        break;
    }
}

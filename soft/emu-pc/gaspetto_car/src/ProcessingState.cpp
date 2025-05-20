#include "ProcessingState.h"

#include "GaspettoCar.h"
#include "MotorController.h"

#include <Arduino.h>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

#define POWER 17
void ProcessingState::processEvent(Event &evt)
{
    MotorController *motorController =
            static_cast<GaspettoCar *>(active_object)->getMotorController();

    if (!motorController) {
        Serial.println("Motor controller not initialized.\n");
        return;
    }

    switch (evt.getCommand()) {
    case CommandId::MOTOR_FORWARD:
        /* Moving forward */
        Serial.println("Moving forward...\n");
        motorController->SetMotor(FORWARD, POWER, 150, FORWARD, POWER, 150);
        break;
    case CommandId::MOTOR_BACKWARD:
        /* Moving backward */
        Serial.println("Moving backward...\n");
        motorController->SetMotor(BACKWARD, POWER, 150, BACKWARD, POWER, 150);
        break;
    case CommandId::MOTOR_RIGHT:
        /* Turning right */
        Serial.println("Turning right...\n");
        motorController->SetMotor(FORWARD, POWER, 150, BACKWARD, POWER, 70);
        break;
    case CommandId::MOTOR_LEFT:
        /* Turning left */
        Serial.println("Turning left...\n");
        motorController->SetMotor(BACKWARD, POWER, 70, FORWARD, POWER, 150);
        break;
    case CommandId::MOTOR_STOP:
        /* Stopping and transitioning to IDLE state */
        Serial.println("Stopping...\n");
        motorController->stopMotorLeft();
        motorController->stopMotorRight();
        active_object->transitionTo(StateId::IDLE);
        break;
    default:
        /* Unknown event in PROCESSING state */
        Serial.println("Unknown event in PROCESSING state.\n");
        assert(true);
        break;
    }
}

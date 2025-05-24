#include "ProcessingState.h"

#include "GaspettoCar.h"
#include "MotorController.h"

#include <Arduino.h>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

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
        motorController->SetMotor(FORWARD, MOTOR_PWM, DISTANCE_CM_FWD_BWD, FORWARD, MOTOR_PWM,
                                  DISTANCE_CM_FWD_BWD);
        break;
    case CommandId::MOTOR_BACKWARD:
        /* Moving backward */
        motorController->SetMotor(BACKWARD, MOTOR_PWM, DISTANCE_CM_FWD_BWD, BACKWARD, MOTOR_PWM,
                                  DISTANCE_CM_FWD_BWD);
        break;
    case CommandId::MOTOR_RIGHT:
        /* Turning right */
        motorController->SetMotor(FORWARD, MOTOR_PWM, DISTANCE_CM_FWD_BWD, BACKWARD, MOTOR_PWM,
                                  DISTANCE_CM_TURN_RIGHT);
        break;
    case CommandId::MOTOR_LEFT:
        /* Turning left */
        motorController->SetMotor(BACKWARD, MOTOR_PWM, DISTANCE_CM_TURN_LEFT, FORWARD, MOTOR_PWM,
                                  DISTANCE_CM_FWD_BWD);
        break;
    case CommandId::MOTOR_STOP:
        /* Stopping and transitioning to IDLE state */
        motorController->StopBothMotors();
        active_object->transitionTo(StateId::IDLE);
        break;
    default:
        /* Unknown event in PROCESSING state */
        Serial.println("Unknown event in PROCESSING state.\n");
        assert(true);
        break;
    }
}

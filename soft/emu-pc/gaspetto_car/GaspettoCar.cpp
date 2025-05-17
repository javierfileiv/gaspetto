#include "GaspettoCar.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "MotorController.h"
#include "State.h"

#include <cstdint>

GaspettoCar::GaspettoCar(State *idle, State *running, EventQueue *queue, StateId initial_state,
                         MotorController *motorController, RadioController *radioController)
        : ActiveObject(queue, nullptr)
        , motorController(motorController)
        , radioController(radioController)
{
    InitMachine(StateId::IDLE, idle);
    InitMachine(StateId::PROCESSING, running);
    SetInitialState(initial_state);
}

void GaspettoCar::Init()
{
    ActiveObject::Init();
    if (motorController) {
        motorController->InitMotorPins();
        motorController->InitSpeedSensor();
    }
}

void GaspettoCar::SetMotor(bool forward_motor_left, uint8_t motor_left_speed,
                           uint8_t distance_cm_left, bool forward_motor_right,
                           uint8_t motor_right_speed, uint8_t distance_cm_right)
{
    if (motorController) {
        motorController->SetMotor(forward_motor_left, motor_left_speed, distance_cm_left,
                                  forward_motor_right, motor_right_speed, distance_cm_right);
    }
}

void GaspettoCar::stopMotorRight()
{
    if (motorController) {
        motorController->stopMotorRight();
    }
}

void GaspettoCar::stopMotorLeft()
{
    if (motorController) {
        motorController->stopMotorLeft();
    }
}

bool GaspettoCar::isTargetReached()
{
    if (!motorController)
        return true;
    return motorController->isTargetReached(currentStateId);
}

int GaspettoCar::postEvent(Event evt)
{
    if (eventQueue) {
        eventQueue->enqueue(evt);
        return 0;
    }
    return -1;
}

void GaspettoCar::processNextEvent()
{
    if (isTargetReached()) {
        if (eventQueue && !eventQueue->IsEmpty()) {
            Event evt;

            State *currentState = states[static_cast<uint8_t>(currentStateId)];
            eventQueue->dequeue(evt);
            currentState->processEvent(evt);
        }
    }
}

void GaspettoCar::enterLowPowerMode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    Serial.println("Entering low-power mode...\n");
    lowPowerMode = true;
    while (lowPowerMode) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); /*  Simulate
                                        low-power
                                        sleep. */
    }
#else
    /*  Implement low-power mode for Arduino here. */
    /*  STM32 sleep modes or power-saving features. */
    delay(100); /*  Simulate low-power sleep. */
#endif
#endif
}

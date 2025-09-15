#pragma once

#include "mock_base.h"
#ifndef MOCK_MOTOR_CONTROL_H
#define MOCK_MOTOR_CONTROL_H

#include "MotorControl.h"

#include <gmock/gmock.h>

class MockMotorControl : public MockBase<MockMotorControl>, public MotorControl {
public:
    MockMotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB);
    ~MockMotorControl();

    MOCK_METHOD(void, _init, (uint32_t pwm_freq));
    MOCK_METHOD(void, _setMotorSpeeds,
                (uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward, bool rightForward));
    MOCK_METHOD(void, _setPWMfrequency, (MotorSide side, uint32_t frequency));
    MOCK_METHOD(void, _setPWMdutyCycle, (MotorSide side, PinPerSide pin, uint32_t percent_duty));
    MOCK_METHOD(void, _stopRightMotor, ());
    MOCK_METHOD(void, _stopLeftMotor, ());
    MOCK_METHOD(void, _stopBothMotors, ());

private:
    static MockMotorControl *instance_;
};

#endif // MOCK_MOTOR_CONTROL_H

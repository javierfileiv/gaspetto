#pragma once

#ifndef MOCK_MOTOR_CONTROL_H
#define MOCK_MOTOR_CONTROL_H

#include "../../targets/gaspetto_car/movement_controller/include/MotorControl.h"

// Simple stub implementation of MotorControl for testing
class StubMotorControl : public MotorControl {
public:
    StubMotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB)
            : MotorControl(lA, lB, rA, rB)
    {
    }
    virtual ~StubMotorControl() = default;

    virtual void init(uint32_t pwm_freq) override
    {
    }
    virtual void setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                bool rightForward) override
    {
    }
    virtual void setPWMfrequency(MotorSide side, uint32_t frequency) override
    {
    }
    virtual void setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty) override
    {
    }
    virtual void stopRightMotor() override
    {
    }
    virtual void stopLeftMotor() override
    {
    }
    virtual void stopBothMotors() override
    {
    }
};

#endif // MOCK_MOTOR_CONTROL_H

#pragma once

#include "mock_base.h"
#ifndef MOCK_MOTOR_CONTROL_H
#define MOCK_MOTOR_CONTROL_H

#include "MotorControlInterface.h"

#include <gmock/gmock.h>

class MockMotorControl : public MockBase<MockMotorControl>, public MotorControlInterface {
public:
    MockMotorControl();
    ~MockMotorControl();

    MOCK_METHOD(void, _init, (uint32_t pwm_freq));
    MOCK_METHOD(void, _setMotorSpeeds,
                (uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward, bool rightForward));
    MOCK_METHOD(void, _setPWMfrequency, (MotorSide side, uint32_t frequency));
    MOCK_METHOD(void, _setPWMdutyCycle, (MotorSide side, PinPerSide pin, uint32_t percent_duty));
    MOCK_METHOD(void, _stopRightMotor, ());
    MOCK_METHOD(void, _stopLeftMotor, ());
    MOCK_METHOD(void, _stopBothMotors, ());

    virtual void init(uint32_t pwm_freq) override;
    virtual void setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                bool rightForward) override;
    virtual void setPWMfrequency(MotorSide side, uint32_t frequency) override;
    virtual void setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty) override;
    virtual void stopRightMotor() override;
    virtual void stopLeftMotor() override;
    virtual void stopBothMotors() override;

private:
    static MockMotorControl *instance_;
};

#endif // MOCK_MOTOR_CONTROL_H

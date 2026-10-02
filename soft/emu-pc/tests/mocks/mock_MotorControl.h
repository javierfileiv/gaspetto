#pragma once

#include "MotorControlInterface.h"
#include "mock_base.h"

#include <gmock/gmock.h>

class MockMotorControl : public MockBase<MockMotorControl>, public MotorControlInterface {
public:
    MockMotorControl();
    ~MockMotorControl();

    MOCK_METHOD(void, _init, (uint32_t pwm_freq));
    MOCK_METHOD(void, _setMotorSpeeds,
                (uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward, bool rightForward));
    MOCK_METHOD(void, _setPWMfrequency, (uint32_t frequency));
    MOCK_METHOD(void, _setPWMdutyCycle, (MotorSide side, PinPerSide pin, uint32_t percent_duty));
    MOCK_METHOD(void, _stopBothMotors, ());

    virtual void init(uint32_t pwm_freq) override;
    virtual void setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                bool rightForward) override;
    virtual void setPWMfrequency(uint32_t frequency) override;
    virtual void setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty) override;
    virtual void stopBothMotors() override;
};

#include "mock_MotorControl.h"

MockMotorControl::MockMotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB)
        : MockBase<MockMotorControl>()
        , MotorControl(lA, lB, rA, rB)
{
    set_instance(this);
}

MockMotorControl::~MockMotorControl()
{
    clear_instance(this);
}

void MockMotorControl::init(uint32_t pwm_freq)
{
    auto mock = MockMotorControl::get_instance();
    mock->_init(pwm_freq);
}

void MockMotorControl::setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                      bool rightForward)
{
    auto mock = MockMotorControl::get_instance();
    mock->_setMotorSpeeds(leftSpeed, rightSpeed, leftForward, rightForward);
}

void MockMotorControl::setPWMfrequency(MotorSide side, uint32_t frequency)
{
    auto mock = MockMotorControl::get_instance();
    mock->_setPWMfrequency(side, frequency);
}

void MockMotorControl::setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty)
{
    auto mock = MockMotorControl::get_instance();
    mock->_setPWMdutyCycle(side, pin, percent_duty);
}

void MockMotorControl::stopRightMotor()
{
    auto mock = MockMotorControl::get_instance();
    mock->_stopRightMotor();
}

void MockMotorControl::stopLeftMotor()
{
    auto mock = MockMotorControl::get_instance();
    mock->_stopLeftMotor();
}

void MockMotorControl::stopBothMotors()
{
    auto mock = MockMotorControl::get_instance();
    mock->_stopBothMotors();
}

#include "MotorControl.h"

#include "Arduino.h"

MotorControl::MotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB)
{
    motor[LEFT].pin[BWD] = lA;
    motor[LEFT].pin[FWD] = lB;
    motor[RIGHT].pin[BWD] = rA;
    motor[RIGHT].pin[FWD] = rB;
}

MotorControl::~MotorControl()
{
}

void MotorControl::init(uint32_t pwm_freq)
{
    pinMode(motor[LEFT].pin[BWD], OUTPUT);
    pinMode(motor[LEFT].pin[FWD], OUTPUT);
    pinMode(motor[RIGHT].pin[BWD], OUTPUT);
    pinMode(motor[RIGHT].pin[FWD], OUTPUT);

    setPWMfrequency(pwm_freq);

    analogWrite(motor[LEFT].pin[BWD], 0);
    analogWrite(motor[LEFT].pin[FWD], 0);
    analogWrite(motor[RIGHT].pin[BWD], 0);
    analogWrite(motor[RIGHT].pin[FWD], 0);
}

void MotorControl::setMotorLeft(uint8_t speed, bool forward)
{
    if (forward) {
        analogWrite(motor[LEFT].pin[BWD], speed);
        analogWrite(motor[LEFT].pin[FWD], 0);
    } else {
        analogWrite(motor[LEFT].pin[BWD], 0);
        analogWrite(motor[LEFT].pin[FWD], speed);
    }
}

void MotorControl::setMotorRight(uint8_t speed, bool forward)
{
    if (forward) {
        analogWrite(motor[RIGHT].pin[BWD], speed);
        analogWrite(motor[RIGHT].pin[FWD], 0);
    } else {
        analogWrite(motor[RIGHT].pin[BWD], 0);
        analogWrite(motor[RIGHT].pin[FWD], speed);
    }
}

void MotorControl::setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                  bool rightForward)
{
    setMotorLeft(leftSpeed, leftForward);
    setMotorRight(rightSpeed, rightForward);
}

void MotorControl::setPWMfrequency(uint32_t frequency)
{
    analogWriteFrequency(frequency);
}

void MotorControl::setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t duty)
{
    analogWrite(motor[side].pin[pin], duty);
}

void MotorControl::stopLeftMotor()
{
    analogWrite(motor[LEFT].pin[BWD], 0);
    analogWrite(motor[LEFT].pin[FWD], 0);
}

void MotorControl::stopRightMotor()
{
    analogWrite(motor[RIGHT].pin[BWD], 0);
    analogWrite(motor[RIGHT].pin[FWD], 0);
}

void MotorControl::stopBothMotors()
{
    stopLeftMotor();
    stopRightMotor();
}

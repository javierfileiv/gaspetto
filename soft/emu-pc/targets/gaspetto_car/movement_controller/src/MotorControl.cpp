#include "MotorControl.h"

#include "Arduino.h"

MotorControl::MotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB)
        : motor_left_pin_a(lA)
        , motor_left_pin_b(lB)
        , motor_right_pin_a(rA)
        , motor_right_pin_b(rB)
{
}

void MotorControl::init(uint32_t pwm_freq)
{
    pinMode(motor_left_pin_a, OUTPUT);
    pinMode(motor_left_pin_b, OUTPUT);
    pinMode(motor_right_pin_a, OUTPUT);
    pinMode(motor_right_pin_b, OUTPUT);
    analogWriteFrequency(pwm_freq);
    stopBothMotors();
}

void MotorControl::setMotorLeft(bool forward, uint8_t speed_percent)
{
    if (forward) {
        analogWrite(motor_left_pin_a, speed_percent);
        digitalWrite(motor_left_pin_b, LOW);
    } else {
        digitalWrite(motor_left_pin_a, LOW);
        analogWrite(motor_left_pin_b, speed_percent);
    }
}

void MotorControl::setMotorRight(bool forward, uint8_t speed_percent)
{
    if (forward) {
        analogWrite(motor_right_pin_a, speed_percent);
        digitalWrite(motor_right_pin_b, LOW);
    } else {
        digitalWrite(motor_right_pin_a, LOW);
        analogWrite(motor_right_pin_b, speed_percent);
    }
}

void MotorControl::setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                  bool rightForward)
{
    setMotorLeft(leftForward, leftSpeed);
    setMotorRight(rightForward, rightSpeed);
}

void MotorControl::stopLeftMotor()
{
    digitalWrite(motor_left_pin_a, LOW);
    digitalWrite(motor_left_pin_b, LOW);
}

void MotorControl::stopRightMotor()
{
    digitalWrite(motor_right_pin_a, LOW);
    digitalWrite(motor_right_pin_b, LOW);
}

void MotorControl::stopBothMotors()
{
    stopLeftMotor();
    stopRightMotor();
}

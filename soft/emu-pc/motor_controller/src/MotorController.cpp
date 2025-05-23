#include "MotorController.h"

#include "Arduino.h"

MotorController::MotorController() = default;

void MotorController::setPins(int lA, int lB, int rA, int rB, int sL, int sR)
{
    MOTOR_LEFT_PIN_A = lA;
    MOTOR_LEFT_PIN_B = lB;
    MOTOR_RIGHT_PIN_A = rA;
    MOTOR_RIGHT_PIN_B = rB;
    SPEED_SENSOR_LEFT_PIN = sL;
    SPEED_SENSOR_RIGHT_PIN = sR;
}

void MotorController::InitMotorPins()
{
    pinMode(MOTOR_RIGHT_PIN_A, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN_B, OUTPUT);
    pinMode(MOTOR_LEFT_PIN_A, OUTPUT);
    pinMode(MOTOR_LEFT_PIN_B, OUTPUT);
}

void MotorController::InitSpeedSensor()
{
    pinMode(SPEED_SENSOR_LEFT_PIN, INPUT);
    pinMode(SPEED_SENSOR_RIGHT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT_PIN), left_motor_speed_sensor_irq,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT_PIN), right_motor_speed_sensor_irq,
                    RISING);
}

void MotorController::ResetCounterMotorRight()
{
    motor_right_pulse_count = 0;
    target_pulses_right = 0;
}

void MotorController::ResetCounterMotorLeft()
{
    motor_left_pulse_count = 0;
    target_pulses_left = 0;
}

void MotorController::SetMotor(bool forward_motor_left, uint8_t motor_left_speed,
                               uint8_t distance_cm_left, bool forward_motor_right,
                               uint8_t motor_right_speed, uint8_t distance_cm_right)
{
    ResetCounterMotorRight();
    ResetCounterMotorLeft();
    target_pulses_left = CentimetersToCount(distance_cm_left);
    target_pulses_right = CentimetersToCount(distance_cm_right);
    if (forward_motor_left) {
        analogWrite(MOTOR_LEFT_PIN_A, mapDutyCycle(motor_left_speed));
        analogWrite(MOTOR_LEFT_PIN_B, 0);
    } else {
        analogWrite(MOTOR_LEFT_PIN_A, 0);
        analogWrite(MOTOR_LEFT_PIN_B, mapDutyCycle(motor_left_speed));
    }
    if (forward_motor_right) {
        analogWrite(MOTOR_RIGHT_PIN_A, mapDutyCycle(motor_right_speed));
        analogWrite(MOTOR_RIGHT_PIN_B, 0);
    } else {
        analogWrite(MOTOR_RIGHT_PIN_A, 0);
        analogWrite(MOTOR_RIGHT_PIN_B, mapDutyCycle(motor_right_speed));
    }
#ifndef ARDUINO
    delay(1000);
#endif
}

void MotorController::stopMotorRight() const
{
    analogWrite(MOTOR_RIGHT_PIN_A, 0);
    analogWrite(MOTOR_RIGHT_PIN_B, 0);
}

void MotorController::stopMotorLeft() const
{
    analogWrite(MOTOR_LEFT_PIN_A, 0);
    analogWrite(MOTOR_LEFT_PIN_B, 0);
}

bool MotorController::isTargetReached()
{
    bool right_reached = motor_right_pulse_count >= target_pulses_right;
    bool left_reached = motor_left_pulse_count >= target_pulses_left;

    if (right_reached)
        stopMotorRight();
    if (left_reached)
        stopMotorLeft();
    if (right_reached && left_reached)
        return true;
    return false;
}

void MotorController::left_motor_speed_sensor_irq()
{
    MotorController::getInstance().motor_left_pulse_count++;
}

void MotorController::right_motor_speed_sensor_irq()
{
    MotorController::getInstance().motor_right_pulse_count++;
}

int MotorController::mapDutyCycle(int dutyCycle)
{
    return map(dutyCycle, 0, 100, 0, 255);
}

uint32_t MotorController::CentimetersToCount(float cm)
{
    const float wheeldiameter = 67.0;
    const float stepcount = 20.0;
    const float circumference = (wheeldiameter * 3.14f) / 10.0f;
    float cm_step = circumference / stepcount;
    float f_result = cm / cm_step;
    return (uint32_t)f_result;
}

long MotorController::getRightPulseCount() const
{
    return motor_right_pulse_count;
}

long MotorController::getLeftPulseCount() const
{
    return motor_left_pulse_count;
}

uint32_t MotorController::getTargetPulsesRight() const
{
    return target_pulses_right;
}

uint32_t MotorController::getTargetPulsesLeft() const
{
    return target_pulses_left;
}

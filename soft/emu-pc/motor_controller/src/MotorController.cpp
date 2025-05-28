#include "MotorController.h"

#include "Arduino.h"

MotorController *MotorController::isr_instance = nullptr;

MotorController::MotorController() = default;

void MotorController::setPins(int lA, int lB, int rA, int rB, int sL, int sR)
{
    motor_left_pin_a = lA;
    motor_left_pin_b = lB;
    motor_right_pin_a = rA;
    motor_right_pin_b = rB;
    speed_sensor_left_pin = sL;
    speed_sensor_right_pin = sR;
}

void MotorController::InitMotorPins()
{
    pinMode(motor_left_pin_a, OUTPUT);
    pinMode(motor_left_pin_b, OUTPUT);
    pinMode(motor_right_pin_a, OUTPUT);
    pinMode(motor_right_pin_b, OUTPUT);
    analogWriteFrequency(PWM_FREQ);
}

void MotorController::InitSpeedSensor()
{
    pinMode(speed_sensor_left_pin, INPUT);
    pinMode(speed_sensor_right_pin, INPUT);
    if (isr_instance == nullptr) {
        isr_instance = this; // Set the static instance pointer
    }
    attachInterrupt(digitalPinToInterrupt(speed_sensor_left_pin), left_motor_speed_sensor_irq,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(speed_sensor_right_pin), right_motor_speed_sensor_irq,
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

void MotorController::SetMotor(bool forward_motor_left, uint32_t motor_left_speed,
                               uint32_t distance_cm_left, bool forward_motor_right,
                               uint32_t motor_right_speed, uint32_t distance_cm_right)
{
    ResetCounterMotorRight();
    ResetCounterMotorLeft();
    target_pulses_left = CentimetersToCount(distance_cm_left);
    target_pulses_right = CentimetersToCount(distance_cm_right);
    if (forward_motor_left) {
        analogWrite(motor_left_pin_a, mapDutyCycle(motor_left_speed));
        analogWrite(motor_left_pin_b, 0);
    } else {
        analogWrite(motor_left_pin_a, 0);
        analogWrite(motor_left_pin_b, mapDutyCycle(motor_left_speed));
    }
    if (forward_motor_right) {
        analogWrite(motor_right_pin_a, mapDutyCycle(motor_right_speed));
        analogWrite(motor_right_pin_b, 0);
    } else {
        analogWrite(motor_right_pin_a, 0);
        analogWrite(motor_right_pin_b, mapDutyCycle(motor_right_speed));
    }
}

void MotorController::StopBothMotors()
{
    stopMotorLeft();
    stopMotorRight();
    ResetCounterMotorLeft();
    ResetCounterMotorRight();
}

void MotorController::stopMotorRight()
{
    analogWrite(motor_right_pin_a, 0);
    analogWrite(motor_right_pin_b, 0);
}

void MotorController::stopMotorLeft()
{
    analogWrite(motor_left_pin_a, 0);
    analogWrite(motor_left_pin_b, 0);
}

bool MotorController::isTargetReached()
{
    bool right_reached = motor_right_pulse_count >= target_pulses_right;
    bool left_reached = motor_left_pulse_count >= target_pulses_left;

    if (left_reached)
        stopMotorLeft();
    if (right_reached)
        stopMotorRight();
    if (right_reached && left_reached)
        return true;
    return false;
}

// Global ISRs that use the current instance pointer
void left_motor_speed_sensor_irq()
{
    MotorController::isr_instance->motor_left_pulse_count++;
}

void right_motor_speed_sensor_irq()
{
    MotorController::isr_instance->motor_right_pulse_count++;
}

uint32_t MotorController::mapDutyCycle(int dutyCycle)
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

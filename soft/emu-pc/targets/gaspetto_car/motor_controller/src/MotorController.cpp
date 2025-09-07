#include "MotorController.h"

#include "Arduino.h"
#include "config_motor.h"

MotorController *MotorController::isr_instance = nullptr;

MotorController::MotorController() = default;

void MotorController::setPins(int lA, int lB, int rA, int rB, int sL, int sR)
{
    motor[LEFT].pin[A] = lA;
    motor[LEFT].pin[B] = lB;
    motor[LEFT].speed_sensor_pin = sL;
    motor[RIGHT].pin[A] = rA;
    motor[RIGHT].pin[B] = rB;
    motor[RIGHT].speed_sensor_pin = sR;
}

#define SET_HW_TIMER(pin) \
    new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM))
#define TIME_CHANNEL(pin) STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM))

void MotorController::setPWMfrequency(MotorSide side, uint32_t frequency)
{
    motor[side].timer->setPWM(motor[side].tim_channel[A], motor[side].pin[A], frequency, 0);
    motor[side].timer->setPWM(motor[side].tim_channel[B], motor[side].pin[B], frequency, 0);
}

void MotorController::setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty)
{
    motor[side].timer->setCaptureCompare(motor[side].tim_channel[pin], percent_duty,
                                         PERCENT_COMPARE_FORMAT);
}

void MotorController::InitMotorPins()
{
    pinMode(motor[LEFT].pin[A], OUTPUT);
    pinMode(motor[LEFT].pin[B], OUTPUT);
    pinMode(motor[RIGHT].pin[A], OUTPUT);
    pinMode(motor[RIGHT].pin[B], OUTPUT);
    motor[LEFT].timer = SET_HW_TIMER(motor[LEFT].pin[A]);
    motor[LEFT].tim_channel[A] = TIME_CHANNEL(motor[LEFT].pin[A]);
    motor[LEFT].tim_channel[B] = TIME_CHANNEL(motor[LEFT].pin[B]);
    motor[RIGHT].timer = SET_HW_TIMER(motor[RIGHT].pin[A]);
    motor[RIGHT].tim_channel[A] = TIME_CHANNEL(motor[RIGHT].pin[A]);
    motor[RIGHT].tim_channel[B] = TIME_CHANNEL(motor[RIGHT].pin[B]);
    setPWMfrequency(LEFT, PWM_FREQ);
    setPWMfrequency(RIGHT, PWM_FREQ);
}

void MotorController::InitSpeedSensor()
{
    pinMode(motor[LEFT].speed_sensor_pin, INPUT);
    pinMode(motor[RIGHT].speed_sensor_pin, INPUT);
    if (isr_instance == nullptr) {
        isr_instance = this; // Set the static instance pointer
    }
    attachInterrupt(digitalPinToInterrupt(motor[LEFT].speed_sensor_pin),
                    left_motor_speed_sensor_irq, RISING);
    attachInterrupt(digitalPinToInterrupt(motor[RIGHT].speed_sensor_pin),
                    right_motor_speed_sensor_irq, RISING);
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
        setPWMdutyCycle(LEFT, A, mapDutyCycle(motor_left_speed));
        setPWMdutyCycle(LEFT, B, 0);
    } else {
        setPWMdutyCycle(LEFT, A, 0);
        setPWMdutyCycle(LEFT, B, mapDutyCycle(motor_left_speed));
    }
    if (forward_motor_right) {
        setPWMdutyCycle(RIGHT, A, mapDutyCycle(motor_right_speed));
        setPWMdutyCycle(RIGHT, B, 0);
    } else {
        setPWMdutyCycle(RIGHT, A, 0);
        setPWMdutyCycle(RIGHT, B, mapDutyCycle(motor_right_speed));
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
    setPWMdutyCycle(RIGHT, A, 0);
    setPWMdutyCycle(RIGHT, B, 0);
}

void MotorController::stopMotorLeft()
{
    setPWMdutyCycle(LEFT, A, 0);
    setPWMdutyCycle(LEFT, B, 0);
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

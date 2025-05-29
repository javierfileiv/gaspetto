#include "MovementController.h"

#include "Arduino.h"

MovementController *MovementController::isr_instance = nullptr;

MovementController::MovementController(MotorControl &motorController,
                                       uint32_t speed_sensor_left_pin,
                                       uint32_t speed_sensor_right_pin)
        : _motorControl(motorController)
        , speed_sensor_left_pin(speed_sensor_left_pin)
        , speed_sensor_right_pin(speed_sensor_right_pin)
        , motor_left_pulse_count(0)
        , motor_right_pulse_count(0)
        , target_pulses_left(0)
        , target_pulses_right(0)
{
    if (isr_instance == nullptr) {
        isr_instance = this;
    } else {
        logln(F("WARNING: MovementController::isr_instance already set! Ensure singleton behavior."));
    }
}

void MovementController::init(uint32_t pwm_freq)
{
    _motorControl.init(pwm_freq);
    pinMode(speed_sensor_left_pin, INPUT);
    pinMode(speed_sensor_right_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(speed_sensor_left_pin), left_motor_speed_sensor_irq,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(speed_sensor_right_pin), right_motor_speed_sensor_irq,
                    RISING);
}

void MovementController::setMotor(bool forward_motor_left, uint32_t motor_left_speed,
                                  uint32_t distance_cm_left, bool forward_motor_right,
                                  uint32_t motor_right_speed, uint32_t distance_cm_right)
{
    uint32_t _leftPercent = map(motor_left_speed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(motor_right_speed, 0, 100, 0, 255);

    resetCounterMotorRight();
    resetCounterMotorLeft();
    target_pulses_left = CentimetersToCount(distance_cm_left);
    target_pulses_right = CentimetersToCount(distance_cm_right);
    _motorControl.setMotorSpeeds(_leftPercent, _rightPercent, forward_motor_left,
                                 forward_motor_right);
}

bool MovementController::isTargetReached()
{
    bool right_reached = motor_right_pulse_count >= target_pulses_right;
    bool left_reached = motor_left_pulse_count >= target_pulses_left;
    if (left_reached)
        _motorControl.stopLeftMotor();
    if (right_reached)
        _motorControl.stopRightMotor();
    return right_reached && left_reached;
}

void MovementController::stopBothMotors()
{
    _motorControl.setMotorSpeeds(0, 0, false, false);
    resetCounterMotorLeft();
    resetCounterMotorRight();
}

void MovementController::stopMotorLeft()
{
    _motorControl.stopLeftMotor();
}

void MovementController::stopMotorRight()
{
    _motorControl.stopRightMotor();
}

void MovementController::resetCounterMotorLeft()
{
    motor_left_pulse_count = 0;
    target_pulses_left = 0;
}

void MovementController::resetCounterMotorRight()
{
    motor_right_pulse_count = 0;
    target_pulses_right = 0;
}

void MovementController::incrementLeftPulseCount()
{
    motor_left_pulse_count++;
}

void MovementController::incrementRightPulseCount()
{
    motor_right_pulse_count++;
}

long MovementController::getLeftPulseCount() const
{
    return motor_left_pulse_count;
}

long MovementController::getRightPulseCount() const
{
    return motor_right_pulse_count;
}

uint32_t MovementController::getLeftTargetPulses() const
{
    return target_pulses_left;
}

uint32_t MovementController::getRightTargetPulses() const
{
    return target_pulses_right;
}

uint32_t MovementController::CentimetersToCount(float cm)
{
    constexpr float WHEEL_DIAMETER_MM = 67.0;
    constexpr float ENCODER_PPR = 20.0;
    constexpr float PI_VAL = 3.14159265359f;
    constexpr float MM_PER_CM = 10.0f;
    constexpr float circumference_mm = WHEEL_DIAMETER_MM * PI_VAL;
    constexpr float mm_per_pulse = circumference_mm / ENCODER_PPR;
    float total_mm = cm * MM_PER_CM;
    float f_result = total_mm / mm_per_pulse;
    return static_cast<uint32_t>(f_result);
}

void left_motor_speed_sensor_irq()
{
    MovementController::isr_instance->incrementLeftPulseCount();
}

void right_motor_speed_sensor_irq()
{
    MovementController::isr_instance->incrementRightPulseCount();
}

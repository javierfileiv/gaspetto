#include "MovementController.h"

#include "Arduino.h"

MovementController *MovementController::isr_instance = nullptr;

MovementController::MovementController(MotorControl &motorControl, uint32_t speed_sensor_left_pin,
                                       uint32_t speed_sensor_right_pin)
        : _motorControl(motorControl)
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
}

void MovementController::setMotor(bool forward_motor_left, uint32_t motor_left_speed,
                                  uint32_t distance_cm_left, bool forward_motor_right,
                                  uint32_t motor_right_speed, uint32_t distance_cm_right)
{
    uint32_t _leftPercent = map(motor_left_speed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(motor_right_speed, 0, 100, 0, 255);

    _motorControl.setMotorSpeeds(_leftPercent, _rightPercent, forward_motor_left,
                                 forward_motor_right);
}

bool MovementController::isTargetReached()
{
    bool right_reached = false;
    bool left_reached = false;
    if (left_reached)
        _motorControl.stopLeftMotor();
    if (right_reached)
        _motorControl.stopRightMotor();
    return right_reached && left_reached;
}

void MovementController::stopBothMotors()
{
    _motorControl.setMotorSpeeds(0, 0, false, false);
}

void MovementController::stopMotorLeft()
{
    _motorControl.stopLeftMotor();
}

void MovementController::stopMotorRight()
{
    _motorControl.stopRightMotor();
}

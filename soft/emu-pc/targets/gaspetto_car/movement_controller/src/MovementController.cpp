#include "MovementController.h"

#include "Arduino.h"

MovementController::MovementController(MotorControl &motorControl)
        : _motorControl(motorControl)
{
}

void MovementController::init(uint32_t pwm_freq)
{
    _motorControl.init(pwm_freq);
}

void MovementController::setMotor(bool forward_motor_left, uint32_t motor_left_speed,
                                  bool forward_motor_right, uint32_t motor_right_speed,
                                  uint32_t timeout_ms)
{
    uint32_t _leftPercent = map(motor_left_speed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(motor_right_speed, 0, 100, 0, 255);

    _motorControl.setMotorSpeeds(_leftPercent, _rightPercent, forward_motor_left,
                                 forward_motor_right);
}

bool MovementController::isTargetReached()
{
    return false;
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

#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "Log.h"
#include "MotorControl.h"

#include <cstdint>

class MovementController : public Log {
public:
    /**
     * MovementController: constructor for the movement controller
     * @motorControl: reference to the MotorControl instance
     */
    MovementController(MotorControl &motorControl);

    /**
     * init(): initialize the movement controller and configure sensors
     * @pwm_freq: PWM frequency for motor control
     */
    void init(uint32_t pwm_freq);

    /**
     * setMotor: set motor speeds and directions for a specified distance
     * @forward_motor_left: true for left motor forward, false for backward
     * @motor_left_speed: PWM duty cycle (0-100) for left motor
     * @forward_motor_right: true for right motor forward, false for backward
     * @motor_right_speed: PWM duty cycle (0-100) for right motor
     * @timeout_ms: optional timeout in milliseconds to stop motors after this duration. If 0, no
     * timeout.
     */
    void setMotor(bool forward_motor_left, uint32_t motor_left_speed, bool forward_motor_right,
                  uint32_t motor_right_speed, uint32_t timeout_ms = 0);

    /**
     * isTargetReached: check if both motors have reached their target pulse counts
     * @return: true if both motors have reached or exceeded their targets, false otherwise
     */
    bool isTargetReached();

    /**
     * stopBothMotors: stop both motors and reset counters and targets
     */
    void stopBothMotors();

    /**
     * stopMotorLeft: stop the left motor only
     */
    void stopMotorLeft();

    /**
     * stopMotorRight: stop the right motor only
     */
    void stopMotorRight();

public:
    MotorControl &_motorControl;

private:
    // IMUOrientation imu;
    bool imuOk = false;
    double yawSetpoint, currentYaw, motorOffsetOutput;
    double Kp = 2.0, Ki = 0.01, Kd = 0.01;
    // PID &pid;
    float targetYaw = 0.0f;
    bool straightDriving = false;
    bool turningInPlace = false;
    float baseSpeed = 0.0f;
    float currentPwmFreq = 17.0f;
    float pidError = 0.0f; // Current PID error for telemetry

    unsigned long straightStartMs = 0;
};

#endif /* MOVEMENT_CONTROLLER_H */

#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "IMUOrientationInterface.h"
#include "MotorControlInterface.h"
#include "PID_v1.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#endif
class MovementController {
public:
    /**
     * MovementController: constructor for the movement controller
     * @motorControl: reference to the MotorControlInterface instance
     * @imu: reference to the IMUOrientationInterface instance
     */
    MovementController(MotorControlInterface &motorControl, IMUOrientationInterface &imu);

    /**
     * init(): initialize the movement controller and configure sensors
     * @pwm_freq: PWM frequency for motor control
     */
    void init(uint32_t pwm_freq);

    /**
     * setMotor: set motor speeds and directions for a specified distance
     * @motor_left_speed: PWM duty cycle (0-100) for left motor
     * @motor_right_speed: PWM duty cycle (0-100) for right motor
     * @forward_motor_left: true for left motor forward, false for backward
     * @forward_motor_right: true for right motor forward, false for backward
     * @timeout_ms: optional timeout in milliseconds to stop motors after this duration. If 0, no
     * timeout.
     */
    void setMotor(uint32_t motor_left_speed, uint32_t motor_right_speed, bool forward_motor_left,
                  bool forward_motor_right, uint32_t timeout_ms = 0);

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

    /**
     * startStraightDriving: begin straight driving with PID control
     * @speed: PWM speed value (positive for forward, negative for backward)
     * @duration_ms: optional duration in milliseconds (0 for unlimited)
     */
    void startStraightDriving(float speed, uint32_t duration_ms = 0);

    /**
     * startTurningInPlace: begin turning in place to a target yaw
     * @target_yaw: target yaw angle in degrees
     * @speed: base speed for turning
     * @duration_ms: optional duration in milliseconds (0 for unlimited)
     */
    void startTurningInPlace(float target_yaw, float speed, uint32_t duration_ms = 0);

    /**
     * updateMovement: update PID control and motor outputs (call from main loop)
     */
    void updateMovement();

    /**
     * stopMovement: stop all PID-controlled movement
     */
    void stopMovement();

    /**
     * isMoving: check if currently performing PID-controlled movement
     */
    bool isMoving() const;

    /**
     * getCurrentError: get current PID error for telemetry
     */
    float getCurrentError() const;

public:
    MotorControlInterface &_motorControl;
    IMUOrientationInterface &_imu;

private:
    // PID library variables (Input, Output, Setpoint)
    double g_yawInput = 0.0;
    double g_yawOutput = 0.0;
    double g_yawSetpoint = 0.0;

    double g_Kp = 2.0;
    double g_Ki = 0.01;
    double g_Kd = 0.01;
    PID g_yawPID;

    bool imuOk = false;
    double yawSetpoint;
    double currentYaw;
    double motorOffsetOutput;
    double Kp = 2.0;
    double Ki = 0.01;
    double Kd = 0.01;
    float targetYaw = 0.0f;
    bool straightDriving = false;
    bool turningInPlace = false;
    float baseSpeed = 0.0f;
    float currentPwmFreq = 17.0f;
    float pidError = 0.0f;
    // Turn completion / anti-oscillation helpers
    int g_stableCount = 0;
    const int STABLE_REQUIRED = 4; // Need N consecutive cycles inside deadband
    const float TURN_DEADBAND_DEG = 3.0f; // Final acceptance window
    const float TURN_DECEL_ANGLE_DEG = 35.0f; // Start reducing speed under this error
    const float TURN_MIN_SPEED_RATIO = 0.28f; // Minimum fraction of base speed

    // Output smoothing to reduce motor jitter
    double g_prevFilteredOutput = 0.0;
    const double OUTPUT_FILTER_ALPHA = 0.25; // 0..1 (higher = more weight to new sample)

    // Timed movement variables
    unsigned long movementDurationMs = 0;
    unsigned long movementStartMs = 0;
    bool timedMovement = false;

    uint32_t leftTargetPulses = 0;
    uint32_t rightTargetPulses = 0;
    float yawDiff(float target, float current);
    void applyPidOutput();
    void checkMovementTimeout();
};

#endif /* MOVEMENT_CONTROLLER_H */

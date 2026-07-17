#pragma once

#include "IMUOrientationInterface.h"
#include "MotorControlInterface.h"
#include "MovementControllerInterface.h"
#include "RadioProtocol.h"

class MovementController : public MovementControllerInterface {
public:
    enum class MovementState { IDLE, STRAIGHT_DRIVING, TURNING_IN_PLACE };
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
    void init(uint32_t pwm_freq) override;

    /**
     * stopBothMotors: stop both motors and reset counters and targets
     */
    void stopBothMotors() override;

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
    void startStraightDriving(float speed, uint32_t duration_ms = 0) override;

    /**
     * startTurningInPlace: begin turning in place to a target yaw
     * @target_yaw: target yaw angle in degrees
     * @speed: base speed for turning
     * @duration_ms: optional duration in milliseconds (0 for unlimited)
     */
    void startTurningInPlace(float target_yaw, float speed, uint32_t duration_ms = 0) override;

    /**
     * updateMovement: update PID control and motor outputs (call from main loop)
     */
    void updateMovement() override;

    /**
     * stopMovement: stop all PID-controlled movement
     */
    void stopMovement();

    /**
     * isMoving: check if currently performing PID-controlled movement
     */
    bool isMoving() const override;

    /**
     * setTelemetryCallback: set callback for sending telemetry
     * @param callback: function to call when telemetry should be sent
     */
    void setTelemetryCallback(void (*callback)(const TelemetryPacket &));

    void setTunings(double kp, double ki, double kd);

    void setTrim(float trim);

    double getKp() const
    {
        return Kp;
    }
    double getKi() const
    {
        return Ki;
    }
    double getKd() const
    {
        return Kd;
    }

    /**
     * buildTelemetryPacket: fill a telemetry packet with current state
     */
    TelemetryPacket buildTelemetryPacket() const;

    /**
     * isImuOk: check if IMU is initialized correctly
     */
    bool isImuOk() const
    {
        return imuOk;
    }

public:
    MotorControlInterface &_motorControl;
    IMUOrientationInterface &_imu;

private:
    bool imuOk = false;
    double yawSetpoint, currentYaw, motorOffsetOutput;
    double Kp = 2.0, Ki = 0.0, Kd = 0.0;
    float telemTargetYaw = 0.0f;
    MovementState currentState = MovementState::IDLE;
    float baseSpeed = 0.0f;
    float telemPwmFreq = 17.0f;
    float pidError = 0.0f; /* Current PID error for telemetry. */
    float trimOffset = 0.0f;

    unsigned long straightStartMs = 0;

    /* Timed movement variables. */
    unsigned long movementDurationMs = 0;
    unsigned long movementStartMs = 0;
    bool timedMovement = false;

    /* Target pulse counts for motors. */
    uint32_t leftTargetPulses = 0;
    uint32_t rightTargetPulses = 0;

    /* Telemetry callback and timing. */
    void (*telemetryCallback)(const TelemetryPacket &) = nullptr;
    unsigned long lastTelemetryMs = 0;
    static constexpr unsigned long telemetryIntervalMs = 500; /* Send telemetry every 500ms. */

    /* PID helper methods. */
    float yawDiff(float target, float current);
    void applyPidOutput();
    void checkTelemetry();
};

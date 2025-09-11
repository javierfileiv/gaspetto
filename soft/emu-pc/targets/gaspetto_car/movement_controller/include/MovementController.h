#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "Log.h"
#include "MotorControl.h"

#include <cstdint>

class MovementController : public Log {
public:
    static MovementController *isr_instance;

    /**
     * MovementController: constructor for the movement controller
     * @motorControl: reference to the MotorControl instance
     * @speed_sensor_left_pin: pin number for the left speed sensor
     * @speed_sensor_right_pin: pin number for the right speed sensor
     */
    MovementController(MotorControl &motorControl, uint32_t speed_sensor_left_pin,
                       uint32_t speed_sensor_right_pin);

    /**
     * init(): initialize the movement controller and configure sensors
     * @pwm_freq: PWM frequency for motor control
     */
    void init(uint32_t pwm_freq);

    /**
     * setMotor: set motor speeds and directions for a specified distance
     * @forward_motor_left: true for left motor forward, false for backward
     * @motor_left_speed: PWM duty cycle (0-100) for left motor
     * @distance_cm_left: target distance in centimeters for left motor
     * @forward_motor_right: true for right motor forward, false for backward
     * @motor_right_speed: PWM duty cycle (0-100) for right motor
     * @distance_cm_right: target distance in centimeters for right motor
     */
    void setMotor(bool forward_motor_left, uint32_t motor_left_speed, uint32_t distance_cm_left,
                  bool forward_motor_right, uint32_t motor_right_speed, uint32_t distance_cm_right);

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
     * resetCounterMotorLeft: reset the left motor pulse counter and target
     */
    void resetCounterMotorLeft();

    /**
     * resetCounterMotorRight: reset the right motor pulse counter and target
     */
    void resetCounterMotorRight();

    /**
     * incrementLeftPulseCount: increment the left motor pulse count (for ISR)
     */
    void incrementLeftPulseCount();

    /**
     * incrementRightPulseCount: increment the right motor pulse count (for ISR)
     */
    void incrementRightPulseCount();

    /**
     * getLeftPulseCount: get current left motor pulse count
     * @return: current pulse count for left motor
     */
    long getLeftPulseCount() const;

    /**
     * getRightPulseCount: get current right motor pulse count
     * @return: current pulse count for right motor
     */
    long getRightPulseCount() const;

    /**
     * getLeftTargetPulses: get target pulse count for left motor
     * @return: target pulse count for left motor
     */
    uint32_t getLeftTargetPulses() const;

    /**
     * getRightTargetPulses: get target pulse count for right motor
     * @return: target pulse count for right motor
     */
    uint32_t getRightTargetPulses() const;

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

    // Target pulse counts for motors
    uint32_t leftTargetPulses = 0;
    uint32_t rightTargetPulses = 0;
};

#endif /* MOVEMENT_CONTROLLER_H */

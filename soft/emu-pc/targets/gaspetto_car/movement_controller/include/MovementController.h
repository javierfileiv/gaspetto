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
     * @leftPin: pin number for the left speed sensor
     * @rightPin: pin number for the right speed sensor
     */
    MovementController(MotorControl &motorControl, uint32_t leftPin, uint32_t rightPin);

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
     * resetCounterMotorLeft: reset the left motor's pulse counter and target
     */
    void resetCounterMotorLeft();

    /**
     * resetCounterMotorRight: reset the right motor's pulse counter and target
     */
    void resetCounterMotorRight();

    /**
     * incrementLeftPulseCount: increment the left motor's pulse count (for ISR use)
     */
    void incrementLeftPulseCount();

    /**
     * incrementRightPulseCount: increment the right motor's pulse count (for ISR use)
     */
    void incrementRightPulseCount();

    /**
     * getLeftPulseCount: get the current left motor pulse count
     * returns the current left motor pulse count
     */
    long getLeftPulseCount() const;

    /**
     * getRightPulseCount: get the current right motor pulse count
     * returns the current right motor pulse count
     */
    long getRightPulseCount() const;

    /**
     * getLeftTargetPulses: get the target left motor pulse count
     * returns the target left motor pulse count
     */
    uint32_t getLeftTargetPulses() const;

    /**
     * getRightTargetPulses: get the target right motor pulse count
     * returns the target right motor pulse count
     */
    uint32_t getRightTargetPulses() const;

private:
    /**
     * CentimetersToCount: convert centimeters to pulse counts based on wheel circumference
     * @cm: distance in centimeters
     * @return: equivalent pulse count for the specified distance
     */
    uint32_t CentimetersToCount(float cm);

public:
    MotorControl &_motorControl;

private:
    volatile long motor_left_pulse_count;
    volatile long motor_right_pulse_count;
    uint32_t target_pulses_left;
    uint32_t target_pulses_right;
    uint32_t speed_sensor_left_pin;
    uint32_t speed_sensor_right_pin;
};

void left_motor_speed_sensor_irq();
void right_motor_speed_sensor_irq();

#endif /* MOVEMENT_CONTROLLER_H */

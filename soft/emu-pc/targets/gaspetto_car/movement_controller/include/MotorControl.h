#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Log.h"

#include <cstdint>

enum MotorSide { LEFT, RIGHT, MAX_SIDES };
enum PinPerSide { BWD, FWD, MAX_PIN };

class HardwareTimer;

struct MotorConfig {
    uint32_t pin[MAX_PIN];
    uint32_t tim_channel[MAX_PIN];
    HardwareTimer *timer;
};

class MotorControl : public Log {
public:
    /** setPins():
     * Set the pins for the motor controller.
     * @lA: Left motor A pin.
     * @lB: Left motor B pin.
     * @rA: Right motor A pin.
     * @rB: Right motor B pin.
     */
    MotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB);

    virtual ~MotorControl() = default;

    /** init():
     * Initialize the motor pins.
     * @pwm_freq: Frequency for the PWM signal.
     */
    virtual void init(uint32_t pwm_freq);

    /** setMotorSpeeds():
     * Set the motor speeds and directions.
     * @leftSpeed: Speed for the left motor.
     * @rightSpeed: Speed for the right motor.
     * @leftForward: Direction for the left motor (true for forward, false for backward).
     * @rightForward: Direction for the right motor (true for forward, false for backward).
     */
    virtual void setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                bool rightForward);

    /* setPWMfrequency():
     * Set the PWM frequency for the motors.
     * @side: Motor side (LEFT or RIGHT).
     * @frequency: Frequency in Hz.
     */
    virtual void setPWMfrequency(MotorSide side, uint32_t frequency);

    /* setPWMdutyCycle():
     * Set the PWM duty cycle for a specific motor pin.
     * @side: Motor side (LEFT or RIGHT).
     * @pin: Pin on the motor side (A or B).
     * @percent_duty: Duty cycle percentage (0-100).
     */
    virtual void setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty);

    /** stopRightMotor():
     * Stop the right motor.
     * Set speed of the right motor to zero and stops it.
     */
    virtual void stopRightMotor();

    /** stopLeftMotor():
     * Stop the left motor.
     * Set speed of the left motor to zero and stops it.
     */
    virtual void stopLeftMotor();

    /** stopBothMotors():
     * Stop both motors.
     * Set speed of both motors to zero and stops them.
     */
    virtual void stopBothMotors();

public:
    struct MotorConfig motor[MAX_SIDES];

private:
    /** setMotorLeft():
     * Set the left motor's direction and speed (PWM duty cycle 0-100).
     * @speed_percent: Speed percentage (0-100).
     * @forward: true for forward, false for backward.
     */
    void setMotorLeft(uint8_t speed_percent, bool forward);

    /** setMotorRight():
     * Set the right motor's direction and speed (PWM duty cycle 0-100).
     * @speed_percent: Speed percentage (0-100).
     * @forward: true for forward, false for backward.
     */
    void setMotorRight(uint8_t speed_percent, bool forward);
};

#endif // MOTOR_CONTROLLER_H

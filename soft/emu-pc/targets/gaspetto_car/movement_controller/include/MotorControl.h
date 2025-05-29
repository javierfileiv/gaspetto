#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Log.h"

#include <cstdint>

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

    /** init():
     * Initialize the motor pins.
     * @pwm_freq: Frequency for the PWM signal.
     */
    void init(uint32_t pwm_freq);

    /** setMotorSpeeds():
     * Set the motor speeds and directions.
     * @leftSpeed: Speed for the left motor.
     * @rightSpeed: Speed for the right motor.
     * @leftForward: Direction for the left motor (true for forward, false for backward).
     * @rightForward: Direction for the right motor (true for forward, false for backward).
     */
    void setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                        bool rightForward);

    /** stopRightMotor():
     * Stop the right motor.
     * Set speed of the right motor to zero and stops it.
     */
    void stopRightMotor();

    /** stopLeftMotor():
     * Stop the left motor.
     * Set speed of the left motor to zero and stops it.
     */
    void stopLeftMotor();

    /** stopBothMotors():
     * Stop both motors.
     * Set speed of both motors to zero and stops them.
     */
    void stopBothMotors();

private:
    /** setMotorLeft():
     * Set the left motor's direction and speed (PWM duty cycle 0-100).
     * @forward: true for forward, false for backward.
     * @speed_percent: Speed percentage (0-100).
     */
    void setMotorLeft(bool forward, uint8_t speed_percent);

    /** setMotorRight():
     * Set the right motor's direction and speed (PWM duty cycle 0-100).
     * @forward: true for forward, false for backward.
     * @speed_percent: Speed percentage (0-100).
     */
    void setMotorRight(bool forward, uint8_t speed_percent);

private:
    uint32_t motor_left_pin_a;
    uint32_t motor_left_pin_b;
    uint32_t motor_right_pin_a;
    uint32_t motor_right_pin_b;
    uint32_t pwm_left = 0;
    uint32_t pwm_right = 0;
};

#endif // MOTOR_CONTROLLER_H

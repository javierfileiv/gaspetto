#pragma once

#include "Log.h"
#include "MotorControlInterface.h"

#ifndef ARDUINO
#include <cstdint>
#endif

struct MotorConfig {
    uint32_t pin[MAX_PIN];
};

class MotorControl : public MotorControlInterface, public Log {
public:
    /** MotorControl():
     * Set the pins for the motor controller.
     * @lA: Left motor BWD pin (PWM 0-255).
     * @lB: Left motor FWD pin (PWM 0-255).
     * @rA: Right motor FWD pin (PWM 0-255).
     * @rB: Right motor BWD pin (PWM 0-255).
     */
    MotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB);

    virtual ~MotorControl();

    /** init():
     * Initialize the motor pins.
     * @pwm_freq: Frequency for the PWM signal.
     */
    virtual void init(uint32_t pwm_freq);

    /** setMotorSpeeds():
     * Set the motor speeds and directions.
     * @leftSpeed: Speed for the left motor (0-255).
     * @rightSpeed: Speed for the right motor (0-255).
     * @leftForward: Direction for the left motor (true for forward, false for backward).
     * @rightForward: Direction for the right motor (true for forward, false for backward).
     */
    virtual void setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                bool rightForward);

    /** stopBothMotors():
     * Stop both motors.
     * Set speed of both motors to zero and stops them.
     */
    virtual void stopBothMotors();

public:
    struct MotorConfig motor[MAX_SIDES];

private:
    /** setMotorLeft():
     * Set the left motor's direction and speed.
     * @speed: PWM value (0-255).
     * @forward: true for forward, false for backward.
     */
    void setMotorLeft(uint8_t speed, bool forward);

    /** setMotorRight():
     * Set the right motor's direction and speed.
     * @speed: PWM value (0-255).
     * @forward: true for forward, false for backward.
     */
    void setMotorRight(uint8_t speed, bool forward);

    /** setPWMfrequency():
     * Set the PWM frequency for the motors.
     * @frequency: Frequency in Hz.
     */
    virtual void setPWMfrequency(uint32_t frequency);

    /** setPWMdutyCycle():
     * Set the PWM duty cycle for a specific motor pin.
     * @side: Motor side (LEFT or RIGHT).
     * @pin: Pin on the motor side (BWD or FWD).
     * @duty: PWM duty cycle (0-255).
     */
    virtual void setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t duty);

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
};

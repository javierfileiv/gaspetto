#ifndef MOTOR_CONTROL_INTERFACE_H
#define MOTOR_CONTROL_INTERFACE_H

#include <stdint.h>

enum MotorSide { LEFT, RIGHT, MAX_SIDES };
enum PinPerSide { BWD, FWD, MAX_PIN };

class MotorControlInterface {
public:
    virtual ~MotorControlInterface() = default;

    virtual void init(uint32_t pwm_freq) = 0;
    virtual void setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                bool rightForward) = 0;
    virtual void setPWMfrequency(MotorSide side, uint32_t frequency) = 0;
    virtual void setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty) = 0;
    virtual void stopRightMotor() = 0;
    virtual void stopLeftMotor() = 0;
    virtual void stopBothMotors() = 0;
};

#endif // MOTOR_CONTROL_INTERFACE_H

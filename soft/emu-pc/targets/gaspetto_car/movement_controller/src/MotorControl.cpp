#include "MotorControl.h"

#include "Arduino.h"

#define SET_HW_TIMER(pin) \
    new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM))
#define TIME_CHANNEL(pin) STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM))

MotorControl::MotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB)
{
    motor[LEFT].pin[A] = lA;
    motor[LEFT].pin[B] = lB;
    motor[RIGHT].pin[A] = rA;
    motor[RIGHT].pin[B] = rB;
}

void MotorControl::init(uint32_t pwm_freq)
{
    pinMode(motor[LEFT].pin[A], OUTPUT);
    pinMode(motor[LEFT].pin[B], OUTPUT);
    pinMode(motor[RIGHT].pin[A], OUTPUT);
    pinMode(motor[RIGHT].pin[B], OUTPUT);
    motor[LEFT].timer = SET_HW_TIMER(motor[LEFT].pin[A]);
    motor[LEFT].tim_channel[A] = TIME_CHANNEL(motor[LEFT].pin[A]);
    motor[LEFT].tim_channel[B] = TIME_CHANNEL(motor[LEFT].pin[B]);
    motor[RIGHT].timer = SET_HW_TIMER(motor[RIGHT].pin[A]);
    motor[RIGHT].tim_channel[A] = TIME_CHANNEL(motor[RIGHT].pin[A]);
    motor[RIGHT].tim_channel[B] = TIME_CHANNEL(motor[RIGHT].pin[B]);
    setPWMfrequency(LEFT, pwm_freq);
    setPWMfrequency(RIGHT, pwm_freq);
}

void MotorControl::setMotorLeft(bool forward, uint8_t speed_percent)
{
    if (forward) {
        setPWMdutyCycle(LEFT, A, speed_percent);
        setPWMdutyCycle(LEFT, B, LOW);
    } else {
        setPWMdutyCycle(LEFT, A, LOW);
        setPWMdutyCycle(LEFT, B, speed_percent);
    }
}

void MotorControl::setMotorRight(bool forward, uint8_t speed_percent)
{
    if (forward) {
        setPWMdutyCycle(RIGHT, A, speed_percent);
        setPWMdutyCycle(RIGHT, B, LOW);
    } else {
        setPWMdutyCycle(RIGHT, A, LOW);
        setPWMdutyCycle(RIGHT, B, speed_percent);
    }
}

void MotorControl::setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                  bool rightForward)
{
    setMotorLeft(leftForward, leftSpeed);
    setMotorRight(rightForward, rightSpeed);
}

void MotorControl::setPWMfrequency(MotorSide side, uint32_t frequency)
{
    motor[side].timer->setPWM(motor[side].tim_channel[A], motor[side].pin[A], frequency, 0);
    motor[side].timer->setPWM(motor[side].tim_channel[B], motor[side].pin[B], frequency, 0);
}

void MotorControl::setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty)
{
    motor[side].timer->setCaptureCompare(motor[side].tim_channel[pin], percent_duty,
                                         PERCENT_COMPARE_FORMAT);
}

void MotorControl::stopLeftMotor()
{
    setPWMdutyCycle(LEFT, A, 0);
    setPWMdutyCycle(LEFT, B, 0);
}

void MotorControl::stopRightMotor()
{
    setPWMdutyCycle(RIGHT, A, 0);
    setPWMdutyCycle(RIGHT, B, 0);
}

void MotorControl::stopBothMotors()
{
    stopLeftMotor();
    stopRightMotor();
}

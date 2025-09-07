#include "MotorControl.h"

#include "Arduino.h"

#define SET_HW_TIMER(pin) \
    new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM))
#define TIME_CHANNEL(pin) STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM))

MotorControl::MotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB)
{
    motor[LEFT].pin[BWD] = lA;
    motor[LEFT].pin[FWD] = lB;
    motor[RIGHT].pin[BWD] = rA;
    motor[RIGHT].pin[FWD] = rB;
}

void MotorControl::init(uint32_t pwm_freq)
{
    pinMode(motor[LEFT].pin[BWD], OUTPUT);
    pinMode(motor[LEFT].pin[FWD], OUTPUT);
    pinMode(motor[RIGHT].pin[BWD], OUTPUT);
    pinMode(motor[RIGHT].pin[FWD], OUTPUT);
    motor[LEFT].timer = SET_HW_TIMER(motor[LEFT].pin[BWD]);
    motor[LEFT].timer->pause();
    motor[LEFT].tim_channel[BWD] = TIME_CHANNEL(motor[LEFT].pin[BWD]);
    motor[LEFT].tim_channel[FWD] = TIME_CHANNEL(motor[LEFT].pin[FWD]);
    motor[RIGHT].timer = SET_HW_TIMER(motor[RIGHT].pin[BWD]);
    motor[RIGHT].timer->pause();
    motor[RIGHT].tim_channel[BWD] = TIME_CHANNEL(motor[RIGHT].pin[BWD]);
    motor[RIGHT].tim_channel[FWD] = TIME_CHANNEL(motor[RIGHT].pin[FWD]);
    setPWMfrequency(LEFT, pwm_freq);
    setPWMfrequency(RIGHT, pwm_freq);
    setPWMdutyCycle(LEFT, BWD, LOW);
    setPWMdutyCycle(LEFT, FWD, LOW);
    setPWMdutyCycle(RIGHT, BWD, LOW);
    setPWMdutyCycle(RIGHT, FWD, LOW);
}

void MotorControl::setMotorLeft(uint8_t speed_percent, bool forward)
{
    setPWMdutyCycle(LEFT, BWD, LOW);
    setPWMdutyCycle(LEFT, FWD, LOW);
    if (forward) {
        setPWMdutyCycle(LEFT, BWD, speed_percent);
        setPWMdutyCycle(LEFT, FWD, LOW);
    } else {
        setPWMdutyCycle(LEFT, BWD, LOW);
        setPWMdutyCycle(LEFT, FWD, speed_percent);
    }
}

void MotorControl::setMotorRight(uint8_t speed_percent, bool forward)
{
    setPWMdutyCycle(RIGHT, BWD, LOW);
    setPWMdutyCycle(RIGHT, FWD, LOW);
    if (forward) {
        setPWMdutyCycle(RIGHT, BWD, speed_percent);
        setPWMdutyCycle(RIGHT, FWD, LOW);
    } else {
        setPWMdutyCycle(RIGHT, BWD, LOW);
        setPWMdutyCycle(RIGHT, FWD, speed_percent);
    }
}

void MotorControl::setMotorSpeeds(uint32_t leftSpeed, uint32_t rightSpeed, bool leftForward,
                                  bool rightForward)
{
    setMotorLeft(leftSpeed, leftForward);
    setMotorRight(rightSpeed, rightForward);
}

void MotorControl::setPWMfrequency(MotorSide side, uint32_t frequency)
{
    motor[side].timer->setPWM(motor[side].tim_channel[BWD], motor[side].pin[BWD], frequency, 0);
    motor[side].timer->setPWM(motor[side].tim_channel[FWD], motor[side].pin[FWD], frequency, 0);
}

void MotorControl::setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty)
{
    motor[side].timer->setCaptureCompare(motor[side].tim_channel[pin], percent_duty,
                                         PERCENT_COMPARE_FORMAT);
}

void MotorControl::stopLeftMotor()
{
    setPWMdutyCycle(LEFT, BWD, 0);
    setPWMdutyCycle(LEFT, FWD, 0);
}

void MotorControl::stopRightMotor()
{
    setPWMdutyCycle(RIGHT, BWD, 0);
    setPWMdutyCycle(RIGHT, FWD, 0);
}

void MotorControl::stopBothMotors()
{
    stopLeftMotor();
    stopRightMotor();
}

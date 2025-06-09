#include "MotorControl.h"

#include "Arduino.h"

#define SET_HW_TIMER(pin) \
    new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM))
#define TIME_CHANNEL(pin) STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM))

MotorControl::MotorControl(uint32_t lA, uint32_t lB, uint32_t rA, uint32_t rB)
{
    motor[LEFT].pin[DIR] = lA;
    motor[LEFT].pin[SPEED] = lB;
    motor[RIGHT].pin[DIR] = rA;
    motor[RIGHT].pin[SPEED] = rB;
}

void MotorControl::init(uint32_t pwm_freq)
{
    pinMode(motor[LEFT].pin[DIR], OUTPUT);
    pinMode(motor[LEFT].pin[SPEED], OUTPUT);
    pinMode(motor[RIGHT].pin[DIR], OUTPUT);
    pinMode(motor[RIGHT].pin[SPEED], OUTPUT);
    digitalWrite(motor[LEFT].pin[DIR], LOW);
    digitalWrite(motor[LEFT].pin[SPEED], LOW);
    digitalWrite(motor[RIGHT].pin[DIR], LOW);
    digitalWrite(motor[RIGHT].pin[SPEED], LOW);
    motor[LEFT].timer = SET_HW_TIMER(motor[LEFT].pin[DIR]);
    motor[LEFT].timer->pause();
    motor[LEFT].tim_channel[DIR] = TIME_CHANNEL(motor[LEFT].pin[DIR]);
    motor[LEFT].tim_channel[SPEED] = TIME_CHANNEL(motor[LEFT].pin[SPEED]);
    motor[RIGHT].timer = SET_HW_TIMER(motor[RIGHT].pin[DIR]);
    motor[RIGHT].timer->pause();
    motor[RIGHT].tim_channel[DIR] = TIME_CHANNEL(motor[RIGHT].pin[DIR]);
    motor[RIGHT].tim_channel[SPEED] = TIME_CHANNEL(motor[RIGHT].pin[SPEED]);
    setPWMfrequency(LEFT, pwm_freq);
    setPWMfrequency(RIGHT, pwm_freq);
    setPWMdutyCycle(LEFT, DIR, LOW);
    setPWMdutyCycle(LEFT, SPEED, LOW);
    setPWMdutyCycle(RIGHT, DIR, LOW);
    setPWMdutyCycle(RIGHT, SPEED, LOW);
}

void MotorControl::setMotorLeft(bool forward, uint8_t speed_percent)
{
    if (forward) {
        setPWMdutyCycle(LEFT, DIR, speed_percent);
        setPWMdutyCycle(LEFT, SPEED, LOW);
    } else {
        setPWMdutyCycle(LEFT, DIR, LOW);
        setPWMdutyCycle(LEFT, SPEED, speed_percent);
    }
}

void MotorControl::setMotorRight(bool forward, uint8_t speed_percent)
{
    if (forward) {
        setPWMdutyCycle(RIGHT, DIR, speed_percent);
        setPWMdutyCycle(RIGHT, SPEED, LOW);
    } else {
        setPWMdutyCycle(RIGHT, DIR, LOW);
        setPWMdutyCycle(RIGHT, SPEED, speed_percent);
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
    motor[side].timer->setPWM(motor[side].tim_channel[DIR], motor[side].pin[DIR], frequency, 0);
    motor[side].timer->setPWM(motor[side].tim_channel[SPEED], motor[side].pin[SPEED], frequency, 0);
}

void MotorControl::setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty)
{
    motor[side].timer->setCaptureCompare(motor[side].tim_channel[pin], percent_duty,
                                         PERCENT_COMPARE_FORMAT);
}

void MotorControl::stopLeftMotor()
{
    setPWMdutyCycle(LEFT, DIR, 0);
    setPWMdutyCycle(LEFT, SPEED, 0);
}

void MotorControl::stopRightMotor()
{
    setPWMdutyCycle(RIGHT, DIR, 0);
    setPWMdutyCycle(RIGHT, SPEED, 0);
}

void MotorControl::stopBothMotors()
{
    stopLeftMotor();
    stopRightMotor();
}

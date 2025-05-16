#pragma once

#include "ActiveObject.h"
#include "Arduino.h"
#include "EventQueue.h"
#include "IdleState.h"
#include "ProcessingState.h"

#ifndef ARDUINO
#include <cstdint>
#define __ASSERT_USE_STDERR
#include <assert.h>
#else
#define assert(x)                              \
    do {                                       \
        Serial.print(F("Assertion failed: ")); \
        Serial.print(F(#x));                   \
        Serial.print(F(" in "));               \
        Serial.print(__FILE__);                \
        Serial.print(F(" at line "));          \
        Serial.println(__LINE__);              \
        while (1) {                            \
            delay(1000);                       \
        }                                      \
    } while (0)
#endif
class GaspettoCar : public ActiveObject {
public:
    GaspettoCar(State *idle, State *running, EventQueue *queue, StateId initial_state);

    void Init(void);
    /* Set motor directions. */
    void SetMotor(bool forward_motor_left, uint8_t motor_left_speed, uint8_t distance_cm_left,
                  bool forward_motor_right, uint8_t motor_right_speed, uint8_t distance_cm_right);
    void processNextEvent(void) override;
    void enterLowPowerMode(void) override;
    void stopMotorRight(void);
    void stopMotorLeft(void);
    void ResetCounterMotorRight(void);
    void ResetCounterMotorLeft(void);

private:
    /* Function to map duty cycle percentage to PWM value */
    static int mapDutyCycle(int dutyCycle)
    {
        return map(dutyCycle, 0, 100, 0, 255);
    }
    uint32_t CentimetersToCount(float cm)
    {
        float f_result = cm / cm_step; /* Calculate result as a float. */
        return (uint32_t)f_result;
    }
    void InitMotorPins(void);
    void InitSpeedSensor(void);
    uint32_t CentimetersToStep(float cm);
    bool isTargetReached(void);

public:
    /* Constants for motor control. */
    const int MOTOR_RIGHT_PIN_A = PB0; /* PWM pin for motor right. */
    const int MOTOR_RIGHT_PIN_B = PB1; /* Direction pin for motor right. */
    const int MOTOR_LEFT_PIN_A = PB11; /* Example PWM pin for motor left.  */
    const int MOTOR_LEFT_PIN_B = PB10; /* Direction pin for motor left.  */
    const int SPEED_SENSOR_LEFT_PIN = PA0; /* Pin for left speed/distance sensor. */
    const int SPEED_SENSOR_RIGHT_PIN = PA1; /* Pin for right speed/distance sensor. */
    const float stepcount = 20.00; /* 20 Slots in disk. */
    const float wheeldiameter = 67.0; /* Wheel diameter in millimeters. */
    /* Calculate wheel circumference in cm. */
    const float circumference = (wheeldiameter * 3.14) / 10;
    const float cm_step = circumference / stepcount; /* CM per Step. */

    /* Variables for motor control. */
    long target_pulses_right;
    long target_pulses_left;
};

#pragma once

#include "Event.h"
#include "State.h"

class Event;

#define FORWARD true
#define BACKWARD false

class ProcessingState : public State {
public:
    void enter() override;
    void processEvent(Event evt) override;

private:
    void moveMotor(float distance, bool forward);
    void stopMotor(void);
    /* Function to map duty cycle percentage to PWM value */
    static int mapDutyCycle(int dutyCycle)
    {
        return map(dutyCycle, 0, 100, 0, 255);
    }
    void InitMotorPins(void);
    void InitSpeedSensor(void);
    void Init(void);

public:
    /* Constants for motor control. */
    const int MOTOR_RIGHT_PIN_A = PB0; /* PWM pin for motor right. */
    const int MOTOR_RIGHT_PIN_B = PB1; /* Direction pin for motor right. */
    const int MOTOR_LEFT_PIN_A = PB11; /* Example PWM pin for motor left.  */
    const int MOTOR_LEFT_PIN_B = PB10; /* Direction pin for motor left.  */
    const int SPEED_SENSOR_LEFT_PIN = PA0; /* Pin for left speed/distance sensor. */
    const int SPEED_SENSOR_RIGHT_PIN = PA1; /* Pin for right speed/distance sensor. */
    /* Variables for motor control. */
    bool initialized = false;
};

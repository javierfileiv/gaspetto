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
    /* Constants for motor control */
    const int MOTOR_1_PIN_A = PB1; /* Example PWM pin for motor A */
    const int MOTOR_1_PIN_B = PB0; /* Direction pin for motor A */
    const int MOTOR_2_PIN_A = PB11; /* Example PWM pin for motor B */
    const int MOTOR_2_PIN_B = PB10; /* Direction pin for motor B */
    const int SPEED_SENSOR_MOTOR_A_PIN = 4; /* Pin for speed/distance sensor */
    const int SPEED_SENSOR_MOTOR_B_PIN = 4; /* Pin for speed/distance sensor */
    const float DISTANCE_PER_PULSE = 0.5; /* Distance in cm per sensor pulse */
    const float TARGET_DISTANCE = 5.0; /* Target distance in cm */
    bool initialized = false;
};

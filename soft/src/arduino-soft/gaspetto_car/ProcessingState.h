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
    void moveMotor(float distance, bool forward);
    void stopMotor(void);

private:
    /* Function to map duty cycle percentage to PWM value */
    static int mapDutyCycle(int dutyCycle)
    {
        return map(dutyCycle, 0, 100, 0, 255);
    }

public:
    /* Constants for motor control */
    const int MOTOR_PWM_PIN_A = PB0; /* Example PWM pin for motor A */
    const int MOTOR_PWM_PIN_B = PB10; /* Example PWM pin for motor B */
    const int MOTOR_DIR_PIN_A = PB1; /* Direction pin for motor A */
    const int MOTOR_DIR_PIN_B = PB11; /* Direction pin for motor B */
    const int SPEED_SENSOR_MOTOR_A_PIN = 4; /* Pin for speed/distance sensor */
    const int SPEED_SENSOR_MOTOR_B_PIN = 4; /* Pin for speed/distance sensor */
    const float DISTANCE_PER_PULSE = 0.5; /* Distance in cm per sensor pulse */
    const float TARGET_DISTANCE = 5.0; /* Target distance in cm */
};

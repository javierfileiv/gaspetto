#include "ProcessingState.h"

#include "GaspettoCar.h"

#include <cassert>
#include <iostream>

/* Constants for motor control */
const int MOTOR_PWM_PIN_A = 9; /* Example PWM pin for motor A */
const int MOTOR_PWM_PIN_B = 10; /* Example PWM pin for motor B */
const int MOTOR_DIR_PIN_A = 2; /* Direction pin for motor A */
const int MOTOR_DIR_PIN_B = 3; /* Direction pin for motor B */
const int SPEED_SENSOR_PIN = 4; /* Pin for speed/distance sensor */
const float DISTANCE_PER_PULSE = 0.5; /* Distance in cm per sensor pulse */
const float TARGET_DISTANCE = 5.0; /* Target distance in cm */

/* Function to move the motor a specific distance */
void moveMotor(float distance, bool forward)
{
    float pulsesRequired = distance / DISTANCE_PER_PULSE;
    int pulseCount = 0;

    /* Set motor direction */
    if (forward) {
        digitalWrite(MOTOR_DIR_PIN_A, HIGH);
        digitalWrite(MOTOR_DIR_PIN_B, LOW);
    } else {
        digitalWrite(MOTOR_DIR_PIN_A, LOW);
        digitalWrite(MOTOR_DIR_PIN_B, HIGH);
    }

    /* Start motor with PWM */
    analogWrite(MOTOR_PWM_PIN_A, 128); /* 50% duty cycle */
    analogWrite(MOTOR_PWM_PIN_B, 128);

    /* Wait for the required number of pulses */
    while (pulseCount < pulsesRequired) {
        if (digitalRead(SPEED_SENSOR_PIN) == HIGH) {
            pulseCount++;
            delay(10); /* Debounce delay */
        }
    }

    /* Stop motor */
    analogWrite(MOTOR_PWM_PIN_A, 0);
    analogWrite(MOTOR_PWM_PIN_B, 0);
}

void ProcessingState::processEvent(Event evt)
{
    switch (evt.getCommand()) {
    case CommandId::MOTOR_FORWARD:
        /* Moving forward */
        Serial.println("Moving forward...\n");
        moveMotor(TARGET_DISTANCE, true);
        break;
    case CommandId::MOTOR_BACKWARD:
        /* Moving backward */
        Serial.println("Moving backward...\n");
        moveMotor(TARGET_DISTANCE, false);
        break;
    case CommandId::MOTOR_RIGHT:
        /* Turning right */
        Serial.println("Turning right...\n");
        /* Implement turning logic (e.g., one motor forward, one backward) */
        break;
    case CommandId::MOTOR_LEFT:
        /* Turning left */
        Serial.println("Turning left...\n");
        /* Implement turning logic (e.g., one motor backward, one forward) */
        break;
    case CommandId::MOTOR_STOP:
        /* Stopping and transitioning to IDLE state */
        Serial.println("Stopping...\n");
        analogWrite(MOTOR_PWM_PIN_A, 0);
        analogWrite(MOTOR_PWM_PIN_B, 0);
        state_machine->transitionTo(StateId::IDLE);
        break;
    default:
        /* Unknown event in PROCESSING state */
        Serial.println("Unknown event in PROCESSING state.\n");
        assert(true);
        break;
    }
}

#include "ProcessingState.h"

#include "GaspettoCar.h"

#include <Arduino.h>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

#define DUTY_CYCLE 30

void ProcessingState::InitMotorPins(void)
{
    /* Initialize motor control pins */
    pinMode(MOTOR_RIGHT_PIN_A, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN_B, OUTPUT);
    pinMode(MOTOR_LEFT_PIN_A, OUTPUT);
    pinMode(MOTOR_LEFT_PIN_B, OUTPUT);
}

void ProcessingState::InitSpeedSensor(void)
{
    /* Initialize speed sensor pins */
    pinMode(SPEED_SENSOR_LEFT_PIN, INPUT);
    pinMode(SPEED_SENSOR_RIGHT_PIN, INPUT);
}

void ProcessingState::enter()
{
    if (!initialized) {
        /* Initialize motor control pins */
        InitMotorPins();
        analogWriteFrequency(35); /* Set PWM frequency to 35Hz. */
        /* Initialize speed sensor pins */
        InitSpeedSensor();
        initialized = true;
    }
}

void ProcessingState::stopMotor(void)
{
    Serial.print("Stopping motor...\n");
    /* Stop motor */
    analogWrite(MOTOR_RIGHT_PIN_A, 0);
    analogWrite(MOTOR_RIGHT_PIN_B, 0);
    analogWrite(MOTOR_LEFT_PIN_A, 0);
    analogWrite(MOTOR_LEFT_PIN_B, 0);
}
/* Function to move the motor a specific distance */
void ProcessingState::moveMotor(float distance, bool forward)
{
#ifdef ARDUINO
#ifdef SPEED_SENSOR_ON
    float pulsesRequired = distance / DISTANCE_PER_PULSE;
    int pulseCount = 0;
#endif
#endif

    /* Set motor direction */
    if (forward) {
#ifndef NOT_RIGHT_MOTOR
        analogWrite(MOTOR_RIGHT_PIN_A, mapDutyCycle(DUTY_CYCLE));
        analogWrite(MOTOR_RIGHT_PIN_B, 0);
#endif
#ifndef NOT_LEFT_MOTOR
        analogWrite(MOTOR_LEFT_PIN_A, mapDutyCycle(DUTY_CYCLE));
        analogWrite(MOTOR_LEFT_PIN_B, 0);
#endif
    } else {
#ifndef NOT_RIGHT_MOTOR
        analogWrite(MOTOR_RIGHT_PIN_A, 0);
        analogWrite(MOTOR_RIGHT_PIN_B, mapDutyCycle(DUTY_CYCLE));
#endif
#ifndef NOT_LEFT_MOTOR
        analogWrite(MOTOR_LEFT_PIN_A, 0);
        analogWrite(MOTOR_LEFT_PIN_B, mapDutyCycle(DUTY_CYCLE));
#endif
    }
#ifndef ARDUINO
    delay(1000);
#endif
}

void ProcessingState::processEvent(Event evt)
{
    switch (evt.getCommand()) {
    case CommandId::MOTOR_FORWARD:
        /* Moving forward */
        Serial.println("Moving forward...\n");
        moveMotor(5, FORWARD);
        break;
    case CommandId::MOTOR_BACKWARD:
        /* Moving backward */
        Serial.println("Moving backward...\n");
        moveMotor(5, BACKWARD);
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
        stopMotor();
        state_machine->transitionTo(StateId::IDLE);
        break;
    default:
        /* Unknown event in PROCESSING state */
        Serial.println("Unknown event in PROCESSING state.\n");
        assert(true);
        break;
    }
}

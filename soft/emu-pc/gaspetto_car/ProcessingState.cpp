#include "ProcessingState.h"

#include "GaspettoCar.h"

#include <Arduino.h>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

void ProcessingState::enter()
{
    /* Initialize motor control pins */
    pinMode(MOTOR_PWM_PIN_A, OUTPUT);
    pinMode(MOTOR_PWM_PIN_B, OUTPUT);
    pinMode(MOTOR_DIR_PIN_A, OUTPUT);
    pinMode(MOTOR_DIR_PIN_B, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN_A, LOW);
    digitalWrite(MOTOR_DIR_PIN_B, LOW);
    digitalWrite(MOTOR_DIR_PIN_A, LOW);
    digitalWrite(MOTOR_DIR_PIN_B, LOW);
#ifdef SPEED_SENSOR_ON
    /* Initialize speed sensor pins */
    pinMode(SPEED_SENSOR_MOTOR_A_PIN, INPUT);
    pinMode(SPEED_SENSOR_MOTOR_B_PIN, INPUT);
#endif
    analogWriteFrequency(35); /* Set PWM frequency to 35Hz. */
    stopMotor();
}

void ProcessingState::stopMotor(void)
{
    /* Stop motor */
    analogWrite(MOTOR_PWM_PIN_A, 0);
    analogWrite(MOTOR_PWM_PIN_B, 0);
    digitalWrite(MOTOR_DIR_PIN_A, LOW);
    digitalWrite(MOTOR_DIR_PIN_B, LOW);
}
/* Function to move the motor a specific distance */
void ProcessingState::moveMotor(float distance, bool forward)
{
    float pulsesRequired = distance / DISTANCE_PER_PULSE;
    int pulseCount = 0;

    /* Set motor direction */
    if (forward) {
        digitalWrite(MOTOR_DIR_PIN_A, HIGH);
        digitalWrite(MOTOR_DIR_PIN_B, HIGH);
    } else {
        digitalWrite(MOTOR_DIR_PIN_A, LOW);
        digitalWrite(MOTOR_DIR_PIN_B, LOW);
    }
    /* Start motor with PWM */
    analogWrite(MOTOR_PWM_PIN_A, mapDutyCycle(50)); /* 50% duty cycle */
    analogWrite(MOTOR_PWM_PIN_B, mapDutyCycle(50));
#ifdef ARDUINO
#ifdef SPEED_SENSOR_ON
    /* Wait for the required number of pulses */
    while (pulseCount < pulsesRequired) {
        if (digitalRead(SPEED_SENSOR_PIN) == HIGH) {
            pulseCount++;
            delay(3); /* Debounce delay */
        }
    }
#else
    delay(8000);
#endif
#else
#endif
    /* Stop motor */
    stopMotor();
}

void ProcessingState::processEvent(Event evt)
{
    switch (evt.getCommand()) {
    case CommandId::MOTOR_FORWARD:
        /* Moving forward */
        Serial.println("Moving forward...\n");
        moveMotor(TARGET_DISTANCE, FORWARD);
        break;
    case CommandId::MOTOR_BACKWARD:
        /* Moving backward */
        Serial.println("Moving backward...\n");
        moveMotor(TARGET_DISTANCE, BACKWARD);
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

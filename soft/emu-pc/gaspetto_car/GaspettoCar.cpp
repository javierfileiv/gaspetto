#include "GaspettoCar.h"

#include "ActiveObject.h"
#include "State.h"

#include <cstdint>

static bool turning = false;

/* IRQ pulse counters. */
static volatile long motor_right_pulse_count;
static volatile long motor_left_pulse_count;

bool GaspettoCar::isTargetReached(void)
{
    if (currentStateId != StateId::PROCESSING)
        return true;
    if (motor_right_pulse_count >= target_pulses_right)
        stopMotorRight();
    if (motor_left_pulse_count >= target_pulses_left)
        stopMotorLeft();

    return motor_right_pulse_count >= target_pulses_right &&
           motor_left_pulse_count >= target_pulses_left;
}

void left_motor_speed_irq(void)
{
    /* Increment left motor pulse count */
    motor_right_pulse_count++;
}

void right_motor_speed_irq(void)
{
    /* Increment right motor pulse count */
    motor_left_pulse_count++;
}

GaspettoCar::GaspettoCar(State *idle, State *running, EventQueue *queue, StateId initial_state)
        : ActiveObject(queue, nullptr)
{
    InitMachine(StateId::IDLE, idle);
    InitMachine(StateId::PROCESSING, running);
    SetInitialState(initial_state);
}

void GaspettoCar::InitMotorPins(void)
{
    /* Initialize motor control pins */
    pinMode(MOTOR_RIGHT_PIN_A, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN_B, OUTPUT);
    pinMode(MOTOR_LEFT_PIN_A, OUTPUT);
    pinMode(MOTOR_LEFT_PIN_B, OUTPUT);
}

void GaspettoCar::InitSpeedSensor(void)
{
    /* Initialize speed sensor pins */
    pinMode(SPEED_SENSOR_LEFT_PIN, INPUT);
    pinMode(SPEED_SENSOR_RIGHT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_LEFT_PIN), left_motor_speed_irq, RISING);
    attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_RIGHT_PIN), right_motor_speed_irq, RISING);
}

void GaspettoCar::Init(void)
{
    /* Initialize motor control pins */
    InitMotorPins();
    analogWriteFrequency(35); /* Set PWM frequency to 35Hz. */
    /* Initialize speed sensor pins */
    InitSpeedSensor();
}

void GaspettoCar::ResetCounterMotorRight(void)
{
    motor_right_pulse_count = 0;
    target_pulses_right = 0;
}
void GaspettoCar::ResetCounterMotorLeft(void)
{
    motor_left_pulse_count = 0;
    target_pulses_left = 0;
}

void GaspettoCar::SetMotor(bool forward_motor_left, uint8_t motor_left_speed,
                           uint8_t distance_cm_left, bool forward_motor_right,
                           uint8_t motor_right_speed, uint8_t distance_cm_right)
{
    ResetCounterMotorRight();
    ResetCounterMotorLeft();
    target_pulses_left = CentimetersToCount(distance_cm_left);
    target_pulses_right = CentimetersToCount(distance_cm_right);
    /* Set motor directions */
    if (forward_motor_left) {
        analogWrite(MOTOR_LEFT_PIN_A, mapDutyCycle(motor_left_speed));
        analogWrite(MOTOR_LEFT_PIN_B, 0);
    } else {
        analogWrite(MOTOR_LEFT_PIN_A, 0);
        analogWrite(MOTOR_LEFT_PIN_B, mapDutyCycle(motor_left_speed));
    }
    if (forward_motor_right) {
        analogWrite(MOTOR_RIGHT_PIN_A, mapDutyCycle(motor_right_speed));
        analogWrite(MOTOR_RIGHT_PIN_B, 0);
    } else {
        analogWrite(MOTOR_RIGHT_PIN_A, 0);
        analogWrite(MOTOR_RIGHT_PIN_B, mapDutyCycle(motor_right_speed));
    }
#ifndef ARDUINO
    delay(1000);
#endif
}

void GaspettoCar::stopMotorRight(void)
{
    Serial.print("Stopping motor RIGHT...\n");
    analogWrite(MOTOR_RIGHT_PIN_A, 0);
    analogWrite(MOTOR_RIGHT_PIN_B, 0);
    ResetCounterMotorRight();
}

void GaspettoCar::stopMotorLeft(void)
{
    Serial.print("Stopping motor LEFT...\n");
    analogWrite(MOTOR_LEFT_PIN_A, 0);
    analogWrite(MOTOR_LEFT_PIN_B, 0);
    ResetCounterMotorLeft();
}

void GaspettoCar::processNextEvent(void)
{
    if (isTargetReached()) {
        if (eventQueue && !eventQueue->IsEmpty()) {
            Event evt;

            State *currentState = states[static_cast<int>(currentStateId)];
            eventQueue->dequeue(evt);
            currentState->processEvent(evt);
        }
    }
}

void GaspettoCar::enterLowPowerMode(void)
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    Serial.println("Entering low-power mode...\n");
    lowPowerMode = true;
    while (lowPowerMode) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); /*  Simulate
                                        low-power
                                        sleep. */
    }
#else
    /*  Implement low-power mode for Arduino here. */
    /*  STM32 sleep modes or power-saving features. */
    delay(100); /*  Simulate low-power sleep. */
#endif
#endif
}

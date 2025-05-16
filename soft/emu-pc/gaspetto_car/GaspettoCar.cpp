#include "GaspettoCar.h"

#include "ActiveObject.h"
#include "State.h"

static bool turning = false;
/* Variables for motor control. */
long target_pulses_1;
long target_pulses_2;
static long initial_motor1_pulses;
static long initial_motor2_pulses;
/* IRQ pulse counters. */
static volatile long motor1_pulse_count;
static volatile long motor2_pulse_count;

static inline long fast_abs(long x)
{
    return (x < 0) ? -x : x;
}

static bool isTargetReached(void)
{
    long current_pulses_motor1 = fast_abs(motor1_pulse_count - initial_motor1_pulses);
    long current_pulses_motor2 = fast_abs(motor2_pulse_count - initial_motor2_pulses);

    return current_pulses_motor1 >= target_pulses_1 && current_pulses_motor2 >= target_pulses_2;
}

void left_motor_speed_irq(void)
{
    /* Increment left motor pulse count */
    motor1_pulse_count++;
}

void right_motor_speed_irq(void)
{
    /* Increment right motor pulse count */
    motor2_pulse_count++;
}

uint32_t GaspettoCar::CentimetersToStep(float cm)
{
    float f_result = cm / cm_step; /* Calculate result as a float. */
    return (uint32_t)f_result;
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

void GaspettoCar::SetMotor(bool forward_motor_left, uint8_t motor_left_speed,
                           bool forward_motor_right, uint8_t motor_right_speed)
{
    stopMotor();
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

void GaspettoCar::stopMotor(void)
{
    Serial.print("Stopping motor...\n");
    /* Stop motor */
    analogWrite(MOTOR_RIGHT_PIN_A, 0);
    analogWrite(MOTOR_RIGHT_PIN_B, 0);
    analogWrite(MOTOR_LEFT_PIN_A, 0);
    analogWrite(MOTOR_LEFT_PIN_B, 0);
}

void GaspettoCar::enqueue_random_commands(const uint8_t num_events)
{
    std::srand(time(nullptr));

    for (uint8_t i = 0; i < num_events; ++i) {
        Event event = { EventId::NRF_IRQ, static_cast<CommandId>(rand() % 4) }; /*  Random
                                                                                   event. */

        postEvent(event);
#ifdef LOW_POWER_MODE
        lowPowerMode = false; /*  Wake up the system. */
#endif
    }
}

void GaspettoCar::processNextEvent(void)
{
    if (isTargetReached()) {
        Serial.println("Target reached.\n");
        return;
    }
    if (eventQueue && !eventQueue->IsEmpty()) {
        Event evt;

        State *currentState = states[static_cast<int>(currentStateId)];
        eventQueue->dequeue(evt);
        currentState->processEvent(evt);
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

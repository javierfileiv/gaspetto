#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "State.h"

#include <Arduino.h>
#include <cstdint>
#include <stdint.h>

#define FORWARD true
#define BACKWARD false
#define MOTOR_PWM 15
#define DISTANCE_CM_FWD_BWD 50
#define DISTANCE_CM_TURN_RIGHT 15
#define DISTANCE_CM_TURN_LEFT 20

const uint32_t PWM_FREQ = 35; /* Set PWM frequency to 35Hz. */
const uint32_t MOTOR_RIGHT_PIN_A = PB0; /* PWM pin for motor right. */
const uint32_t MOTOR_RIGHT_PIN_B = PB1; /* Direction pin for motor right. */
const uint32_t MOTOR_LEFT_PIN_A = PB11; /* Example PWM pin for motor left.  */
const uint32_t MOTOR_LEFT_PIN_B = PB10; /* Direction pin for motor left.  */
const uint32_t SPEED_SENSOR_LEFT_PIN = PA1; /* Pin for left speed/distance sensor. */
const uint32_t SPEED_SENSOR_RIGHT_PIN = PA0; /* Pin for right speed/distance sensor. */

class MotorController {
public:
    MotorController();
    MotorController(const MotorController &) = delete;
    MotorController &operator=(const MotorController &) = delete;
    void InitMotorPins();
    void InitSpeedSensor();
    void ResetCounterMotorRight();
    void ResetCounterMotorLeft();
    void SetMotor(bool forward_motor_left, uint32_t motor_left_speed, uint32_t distance_cm_left,
                  bool forward_motor_right, uint32_t motor_right_speed, uint32_t distance_cm_right);
    void StopBothMotors();
    void stopMotorRight();
    void stopMotorLeft();
    bool isTargetReached(void);
    void setPins(int lA, int lB, int rA, int rB, int sL, int sR);

    uint32_t mapDutyCycle(int dutyCycle);
    uint32_t CentimetersToCount(float cm);
    long getRightPulseCount() const;
    long getLeftPulseCount() const;
    uint32_t getTargetPulsesRight() const;
    uint32_t getTargetPulsesLeft() const;

    /* Pointer to the current instance for ISR access. */
    static MotorController *isr_instance;

private:
    friend void left_motor_speed_sensor_irq();
    friend void right_motor_speed_sensor_irq();
    volatile uint32_t motor_right_pulse_count;
    volatile uint32_t motor_left_pulse_count;
    uint32_t target_pulses_right;
    uint32_t target_pulses_left;
    int motor_left_pin_a, motor_left_pin_b, motor_right_pin_a, motor_right_pin_b;
    int speed_sensor_left_pin, speed_sensor_right_pin;
};

void left_motor_speed_sensor_irq();
void right_motor_speed_sensor_irq();

#endif /* MOTOR_CONTROLLER_H */

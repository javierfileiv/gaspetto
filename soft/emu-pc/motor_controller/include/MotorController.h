#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "State.h"

#include <Arduino.h>
#include <cstdint>
#include <stdint.h>

#define FORWARD true
#define BACKWARD false
#define MOTOR_PWM 17
#define DISTANCE_CM_FWD_BWD 150
#define DISTANCE_CM_TURN_RIGHT 50
#define DISTANCE_CM_TURN_LEFT 20

extern const int MOTOR_RIGHT_PIN_A;
extern const int MOTOR_RIGHT_PIN_B;
extern const int MOTOR_LEFT_PIN_A;
extern const int MOTOR_LEFT_PIN_B;
extern const int SPEED_SENSOR_LEFT_PIN;
extern const int SPEED_SENSOR_RIGHT_PIN;

class MotorController {
public:
    MotorController();
    MotorController(const MotorController &) = delete;
    MotorController &operator=(const MotorController &) = delete;
    void InitMotorPins();
    void InitSpeedSensor();
    void ResetCounterMotorRight();
    void ResetCounterMotorLeft();
    void SetMotor(bool forward_motor_left, uint8_t motor_left_speed, uint8_t distance_cm_left,
                  bool forward_motor_right, uint8_t motor_right_speed, uint8_t distance_cm_right);
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
    static void set_isr_instance(MotorController *inst)
    {
        isr_instance = inst;
    }

private:
    friend void left_motor_speed_sensor_irq();
    friend void right_motor_speed_sensor_irq();
    volatile long motor_right_pulse_count;
    volatile long motor_left_pulse_count;
    uint32_t target_pulses_right;
    uint32_t target_pulses_left;
    int motor_left_pin_a, motor_left_pin_b, motor_right_pin_a, motor_right_pin_b;
    int speed_sensor_left_pin, speed_sensor_right_pin;
};

void left_motor_speed_sensor_irq();
void right_motor_speed_sensor_irq();

#endif /* MOTOR_CONTROLLER_H */

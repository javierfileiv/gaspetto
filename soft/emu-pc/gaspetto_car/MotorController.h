#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "State.h"

#include <Arduino.h>
#include <cstdint>
#include <stdint.h>

class MotorController {
public:
    static MotorController &getInstance()
    {
        static MotorController instance;
        return instance;
    }
    void InitMotorPins();
    void InitSpeedSensor();
    void ResetCounterMotorRight();
    void ResetCounterMotorLeft();
    void SetMotor(bool forward_motor_left, uint8_t motor_left_speed, uint8_t distance_cm_left,
                  bool forward_motor_right, uint8_t motor_right_speed, uint8_t distance_cm_right);
    void stopMotorRight() const;
    void stopMotorLeft() const;
    bool isTargetReached(StateId currentStateId);
    void setPins(int lA, int lB, int rA, int rB, int sL, int sR);

private:
    MotorController();
    MotorController(const MotorController &) = delete;
    MotorController &operator=(const MotorController &) = delete;
    static void left_motor_speed_sensor_irq();
    static void right_motor_speed_sensor_irq();
    int mapDutyCycle(int dutyCycle);
    uint32_t CentimetersToCount(float cm);
    long getRightPulseCount() const;
    long getLeftPulseCount() const;
    uint32_t getTargetPulsesRight() const;
    uint32_t getTargetPulsesLeft() const;

private:
    volatile long motor_right_pulse_count;
    volatile long motor_left_pulse_count;
    uint32_t target_pulses_right;
    uint32_t target_pulses_left;
    int MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B;
    int SPEED_SENSOR_LEFT_PIN, SPEED_SENSOR_RIGHT_PIN;
};

#endif

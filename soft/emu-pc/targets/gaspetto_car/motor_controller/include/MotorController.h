#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "HardwareTimer.h"
#include "State.h"
#include "config_motor.h"

#include <Arduino.h>
#include <cstdint>
#include <stdint.h>

enum MotorSide { LEFT, RIGHT, MAX_SIDES };
enum PinPerSide { A, B, MAX_PIN };

struct MotorConfig {
    uint32_t pin[MAX_PIN];
    uint32_t tim_channel[MAX_PIN];
    uint32_t speed_sensor_pin;
    HardwareTimer *timer;
};

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
    void setPWMfrequency(MotorSide side, uint32_t frequency);
    void setPWMdutyCycle(MotorSide side, PinPerSide pin, uint32_t percent_duty);
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

protected:
    struct MotorConfig motor[MAX_SIDES];

private:
    friend void left_motor_speed_sensor_irq();
    friend void right_motor_speed_sensor_irq();
    volatile uint32_t motor_right_pulse_count;
    volatile uint32_t motor_left_pulse_count;
    uint32_t target_pulses_right;
    uint32_t target_pulses_left;
};

void left_motor_speed_sensor_irq();
void right_motor_speed_sensor_irq();

#endif /* MOTOR_CONTROLLER_H */

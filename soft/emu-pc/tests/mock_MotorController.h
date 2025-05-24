#ifndef MOCK_MOTOR_CONTROLLER_H
#define MOCK_MOTOR_CONTROLLER_H
#pragma once
#include "MotorController.h"
#include "mock_arduino.h"

#include <gmock/gmock.h>

class MockMotorController : public MotorController {
public:
    /* Mock methods. */
    uint32_t mapDutyCycle(int dutyCycle);
    uint32_t CentimetersToCount(float cm);
    void expect_motor_controller_init();
    void expect_motor_pins_init();
    void expect_speed_sensor_init();
    void expect_turn_right(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                           uint8_t distance_right);
    void expect_turn_left(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                          uint8_t distance_right);
    void expect_move_forward(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                             uint8_t distance_right);
    void expect_move_backward(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                              uint8_t distance_right);
    void expect_stop_motor_left();
    void expect_stop_motor_right();

    void execute_irq_left(int n_times = 0)
    {
        _mock_arduino.execute_irq_left(n_times);
    }

    void execute_irq_right(int n_times = 0)
    {
        _mock_arduino.execute_irq_right(n_times);
    }

    MockMotorController(testing::StrictMock<MockArduino> &mock_arduino)
            : MotorController()
            , _mock_arduino(mock_arduino)
    {
    }
    MockMotorController(const MockMotorController &) = delete;
    MockMotorController &operator=(const MockMotorController &) = delete;

private:
    testing::StrictMock<MockArduino> &_mock_arduino;
};

#endif /* MOCK_MOTOR_CONTROLLER_H */

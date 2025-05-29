#ifndef GASPETTO_CAR_FIXTURE_H
#define GASPETTO_CAR_FIXTURE_H

#include "Context.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "MovementController.h"
#include "ProcessingState.h"
#include "State.h"
#include "config_radio.h"
#include "mock_Arduino.h"
#include "mock_RadioController.h"

#include <gtest/gtest.h>

using testing::_;
using testing::Eq;

static const uint8_t *test_writing_addr = gaspetto_box_pipe_name;
static const uint8_t *test_reading_addr = gaspetto_car_pipe_name;

void enter_low_power_mode(void);

class Fixture : public ::testing::Test {
public:
    Fixture()
            : mock_RadioController(&eventQueue, test_writing_addr, test_reading_addr, mock_rf24)
            , carMovementController(motorController, SPEED_SENSOR_LEFT_PIN, SPEED_SENSOR_RIGHT_PIN)
            , motorController(MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B, MOTOR_RIGHT_PIN_A,
                              MOTOR_RIGHT_PIN_B)
            , ctx({ &eventQueue, &carMovementController, &mock_RadioController, nullptr, &idleState,
                    &processingState, PWM_FREQ })
            , car(ctx)
    {
    }
    void SetUp() override
    {
        car.setLowPowerModeCallback(enter_low_power_mode);
        expect_car_init();
        car.init(StateId::IDLE);
        ASSERT_EQ(car.getCurrentState(), &idleState);
    }
    void TearDown() override
    {
        /* Clean ISR instance. */
        MovementController::isr_instance = nullptr;
    }

    void expect_car_init();
    void expect_motor_control_init();
    void expect_movement_controller_init();
    void expect_enter_low_power_mode();
    void expect_turn_left(uint32_t leftSpeed, uint32_t rightSpeed);
    void expect_turn_right(uint32_t leftSpeed, uint32_t rightSpeed);
    void expect_move_forward(uint32_t leftSpeed, uint32_t rightSpeed);
    void expect_move_backward(uint32_t leftSpeed, uint32_t rightSpeed);

    void execute_irq_left(int n_times = 0)
    {
        _mock_arduino.execute_irq_left(n_times);
    }

    void execute_irq_right(int n_times = 0)
    {
        _mock_arduino.execute_irq_right(n_times);
    }

    void expect_stop_motor_left();
    void expect_stop_motor_right();
    void expect_both_motors_stop();

    void expect_radio_initialization();
    void expect_radio_process_event(Event *evt = nullptr);

    /* ACTIONS ON ActiveObject. */
    void stop_car();
    void execute_irq_left_only(int n_times = 0)
    {
        _mock_arduino.execute_irq_left(n_times);
    }
    void execute_irq_right_only(int n_times = 0)
    {
        _mock_arduino.execute_irq_right(n_times);
    }

    void execute_irq(int n_times = 0)
    {
        _mock_arduino.execute_irq_left(n_times);
        _mock_arduino.execute_irq_right(n_times);
    }

public:
    Context ctx;
    IdleState idleState;
    ProcessingState processingState;
    EventQueue eventQueue;
    MockRadioController mock_RadioController;
    MovementController carMovementController;
    GaspettoCar car;
    MotorControl motorController;
    testing::StrictMock<MockArduino> _mock_arduino;
    testing::StrictMock<MockRF24> mock_rf24;

    Event forwardEvent{ EventId::ACTION, CommandId::MOTOR_FORWARD };
    Event backwardEvent{ EventId::ACTION, CommandId::MOTOR_BACKWARD };
    Event stopEvent{ EventId::ACTION, CommandId::MOTOR_STOP };
    Event leftEvent{ EventId::ACTION, CommandId::MOTOR_LEFT };
    Event rightEvent{ EventId::ACTION, CommandId::MOTOR_RIGHT };

private:
    ::testing::InSequence seq;
};

#endif /* GASPETTO_CAR_FIXTURE_H */

#include "Event.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "IdleState.h"
#include "MotorController.h"
#include "ProcessingState.h"
#include "RadioController.h"
#include "gmock/gmock.h"
#include "mock_MotorController.h"
#include "mock_RadioController.h"

#include <gtest/gtest.h>

using testing::_;
using testing::Eq;
using testing::Return;

static const uint8_t test_writing_addr[5] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
static const uint8_t test_reading_addr[5] = { 0x06, 0x07, 0x08, 0x09, 0x0A };

class Fixture : public ::testing::Test {
public:
    Fixture()
            : mockMotorController(mock_arduino)
            , mock_RadioController(&eventQueue, test_writing_addr, test_reading_addr, mock_rf24)
            , car(&idleState, &processingState, &eventQueue, &mockMotorController,
                  &mock_RadioController)
    {
    }
    void SetUp() override
    {
        mockMotorController.setPins(MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B, MOTOR_RIGHT_PIN_A,
                                    MOTOR_RIGHT_PIN_B, SPEED_SENSOR_LEFT_PIN,
                                    SPEED_SENSOR_RIGHT_PIN);
    }

    void expect_car_init();
    void expect_motor_pins_init();
    void expect_speed_sensor_init();
    void expect_enter_low_power_mode();
    void expect_turn_right(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                           uint8_t distance_right);
    void expect_turn_left(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                          uint8_t distance_right);
    void expect_move_forward(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                             uint8_t distance_right);
    void expect_move_backward(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                              uint8_t distance_right);
    void expect_both_motors_stop()
    {
        expect_stop_motor_left();
        expect_stop_motor_right();
    }
    void expect_stop_motor_left();
    void expect_stop_motor_right();

    void expect_radio_initialization();
    void expect_radio_process_event(Event *evt = nullptr);

    /* ACTIONS ON ActiveObject. */
    void stop_car();
    void execute_irq_left_only(int n_times = 0)
    {
        mockMotorController.execute_irq_left(n_times);
    }
    void execute_irq_right_only(int n_times = 0)
    {
        mockMotorController.execute_irq_right(n_times);
    }

    void execute_irq(int n_times = 0)
    {
        mockMotorController.execute_irq_left(n_times);
        mockMotorController.execute_irq_right(n_times);
    }

public:
    IdleState idleState;
    ProcessingState processingState;
    EventQueue eventQueue;
    MockRadioController mock_RadioController;
    GaspettoCar car;
    MockMotorController mockMotorController;
    testing::StrictMock<MockArduino> mock_arduino;
    testing::StrictMock<MockRF24> mock_rf24;

    Event forwardEvent{ EventId::ACTION, CommandId::MOTOR_FORWARD };
    Event backwardEvent{ EventId::ACTION, CommandId::MOTOR_BACKWARD };
    Event stopEvent{ EventId::ACTION, CommandId::MOTOR_STOP };
    Event leftEvent{ EventId::ACTION, CommandId::MOTOR_LEFT };
    Event rightEvent{ EventId::ACTION, CommandId::MOTOR_RIGHT };

private:
    ::testing::InSequence seq;
};

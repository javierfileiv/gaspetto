#ifndef GASPETTO_CAR_FIXTURE_H
#define GASPETTO_CAR_FIXTURE_H

#include "Context.h"
#include "Event.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "MovementController.h"
#include "ProcessingState.h"
#include "RadioController.h"
#include "State.h"
#include "config_event.h"
#include "config_radio.h"
#include "mock_Arduino.h"
#include "mock_MotorControl.h"
#include "mock_RF24.h"

#include <gtest/gtest.h>

using testing::_;
using testing::Eq;

static const uint8_t *test_writing_addr = gaspetto_box_pipe_name;
static const uint8_t *test_reading_addr = gaspetto_car_pipe_name;

void enter_low_power_mode(void);

class Fixture : public ::testing::Test {
public:
    Fixture()
            : radioController(_mock_RF24, &eventQueue, test_writing_addr, test_reading_addr)
            , _mock_motorControl(MOTOR_LEFT_FWD, MOTOR_LEFT_BWD, MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD)
            , carMovementController(_mock_motorControl)
            , eventQueue()
            , ctx{ &eventQueue, &carMovementController, &radioController, nullptr,
                   &idleState,  &processingState,       MOTOR_FREQ }
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

    void expect_car_init();
    /* Movement/Motor. */
    void expect_movement_controller_init();
    void expect_motor_control_init();
    void expect_enter_low_power_mode();
    void expect_move_forward(uint32_t leftSpeed, uint32_t rightSpeed);
    void expect_move_backward(uint32_t leftSpeed, uint32_t rightSpeed);
    void expect_turn_left(uint32_t leftSpeed, uint32_t rightSpeed);
    void expect_turn_right(uint32_t leftSpeed, uint32_t rightSpeed);
    void expect_stop_motor_left();
    void expect_stop_motor_right();
    void expect_both_motors_stop();

private:
    void expect_set_motor_left(bool forward, uint8_t speed_percent);
    void expect_set_motor_right(bool forward, uint8_t speed_percent);
    void stop_car();

public:
    /* Radio. */
    void expect_radio_initialization();
    void radio_receive_event(Event *evt = nullptr);
    void expect_transmit_event(Event evt);
    void expect_process_radio_no_event();
    void RxRadioEvent(Event evt);
    void expect_send_event(Event *evt = nullptr);
    void expect_radio_process_event(Event *evt = nullptr);

protected:
    Context ctx;
    GaspettoCar car;
    IdleState idleState;
    ProcessingState processingState;
    Event forwardEvent{ EventId::ACTION, CommandId::MOTOR_FORWARD };
    Event backwardEvent{ EventId::ACTION, CommandId::MOTOR_BACKWARD };
    Event stopEvent{ EventId::ACTION, CommandId::MOTOR_STOP };
    Event leftEvent{ EventId::ACTION, CommandId::MOTOR_LEFT };
    Event rightEvent{ EventId::ACTION, CommandId::MOTOR_RIGHT };

private:
    EventQueue eventQueue;
    RadioController radioController;
    MovementController carMovementController;
    testing::StrictMock<MockMotorControl> _mock_motorControl;
    testing::StrictMock<MockArduino> _mock_arduino;
    testing::StrictMock<MockRF24> _mock_RF24;
    ::testing::InSequence seq;
};

#endif /* GASPETTO_CAR_FIXTURE_H */

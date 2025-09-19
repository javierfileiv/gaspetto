#include "fixture.h"

#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class RadioControllerTest : public Fixture {};

TEST_F(RadioControllerTest, InitOnly)
{
}

class RadioControllerTest_CarInit : public RadioControllerTest {};

TEST_F(RadioControllerTest_CarInit, ForwardEvent)
{
    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Radio receive FWD event. */
    radio_receive_event(&forwardEvent);
    /* Motor moves FWD. */
    expect_move_forward(INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    car.processNextEvent();
}

TEST_F(RadioControllerTest_CarInit, BackwardEvent)
{
    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Radio receive BWD event. */
    radio_receive_event(&backwardEvent);
    /* Motor moves BWD. */
    expect_move_backward(INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    car.processNextEvent();
}

TEST_F(RadioControllerTest_CarInit, DISABLED_TurnLeftEvent)
{
    /* Radio receive TURN LEFT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    radio_receive_event(&leftEvent);
    /* Motor TURNS LEFT. */
    expect_turn_left(TURN_MOTOR_SPEED, INITIAL_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    car.processNextEvent();
    /* Receive nothing. */
    radio_receive_event(nullptr);
}

TEST_F(RadioControllerTest_CarInit, DISABLED_TurnRightEvent)
{
    /* Radio receive TURN RIGHT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    radio_receive_event(&rightEvent);
    /* Motor TURNS RIGHT. */
    expect_turn_right(INITIAL_MOTOR_SPEED, TURN_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    car.processNextEvent();
    /* Receive nothing. */
    radio_receive_event(nullptr);
}

TEST_F(RadioControllerTest_CarInit, StopEvent)
{
    /* Radio receive STOP event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    radio_receive_event(&stopEvent);
    /* Motor receives STOP. */
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &idleState);
}

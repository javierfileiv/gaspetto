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
    expect_stop_motor_left();
    expect_stop_motor_right();
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
    expect_stop_motor_left();
    expect_stop_motor_right();
    car.processNextEvent();
}

TEST_F(RadioControllerTest_CarInit, DISABLED_TurnRightEvent)
{
    uint32_t diff = 0;

    /* Radio receive TURN RIGHT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    radio_receive_event(&rightEvent);
    /* Motor TURNS RIGHT. */
    expect_turn_right(MOTOR_FREQ, MOTOR_FREQ);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor right.*/
    expect_stop_motor_right();
    car.processNextEvent();
    expect_both_motors_stop();
    car.processNextEvent();
    /* Receive nothing. */
    radio_receive_event(nullptr);
}

TEST_F(RadioControllerTest_CarInit, DISABLED_TurnLeftEvent)
{
    uint32_t diff = 0;

    /* Radio receive TURN LEFT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    radio_receive_event(&leftEvent);
    /* Motor TURNS LEFT. */
    expect_turn_left(MOTOR_FREQ, MOTOR_FREQ);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor left.*/
    expect_stop_motor_left();
    car.processNextEvent();
    /* Expect stop motor right.*/
    expect_both_motors_stop();
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

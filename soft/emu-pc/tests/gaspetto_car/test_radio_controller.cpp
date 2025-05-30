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
    expect_receive_event(&forwardEvent);
    ProcessRadio();
    /* Motor moves FWD. */
    expect_move_forward(MOTOR_SPEED, MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(ctx.movementController->getLeftTargetPulses());
    car.processNextEvent();
    expect_receive_event(nullptr);
    ProcessRadio();
}

TEST_F(RadioControllerTest_CarInit, BackwardEvent)
{
    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Radio receive BWD event. */
    expect_receive_event(&backwardEvent);
    ProcessRadio();
    /* Motor moves BWD. */
    expect_move_backward(MOTOR_SPEED, MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(ctx.movementController->getLeftTargetPulses());
    car.processNextEvent();
    /* Receive nothing. */
    expect_receive_event(nullptr);
    ProcessRadio();
}

TEST_F(RadioControllerTest_CarInit, TurnRightEvent)
{
    uint32_t diff = 0;

    /* Radio receive TURN RIGHT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    expect_receive_event(&rightEvent);
    ProcessRadio();
    /* Motor TURNS RIGHT. */
    expect_turn_right(MOTOR_SPEED, MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor right.*/
    expect_stop_motor_right();
    execute_irq(ctx.movementController->getRightTargetPulses());
    car.processNextEvent();
    /* Expect stop motor left.*/
    diff = ctx.movementController->getLeftTargetPulses() -
           ctx.movementController->getRightTargetPulses();
    expect_both_motors_stop();
    execute_irq(diff);
    car.processNextEvent();
    /* Receive nothing. */
    expect_receive_event(nullptr);
    ProcessRadio();
}

TEST_F(RadioControllerTest_CarInit, TurnLeftEvent)
{
    uint32_t diff = 0;

    /* Radio receive TURN LEFT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    expect_receive_event(&leftEvent);
    ProcessRadio();
    /* Motor TURNS LEFT. */
    expect_turn_left(MOTOR_SPEED, MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor left.*/
    expect_stop_motor_left();
    execute_irq(ctx.movementController->getLeftTargetPulses());
    car.processNextEvent();
    /* Expect stop motor right.*/
    diff = ctx.movementController->getRightTargetPulses() -
           ctx.movementController->getLeftTargetPulses();
    expect_both_motors_stop();
    execute_irq(diff);
    car.processNextEvent();
    /* Receive nothing. */
    expect_receive_event(nullptr);
    ProcessRadio();
}

TEST_F(RadioControllerTest_CarInit, StopEvent)
{
    /* Radio receive STOP event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    expect_receive_event(&stopEvent);
    ProcessRadio();
    /* Motor receives STOP. */
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Receive nothing. */
    expect_receive_event(nullptr);
    ProcessRadio();
}

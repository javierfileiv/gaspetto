#include "fixture.h"

#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class MovementControllerTest : public Fixture {};

TEST_F(MovementControllerTest, InitOnly)
{
}

class MovementController_CarInit : public MovementControllerTest {};

TEST_F(MovementController_CarInit, ForwardEvent)
{
    /* Post FWD event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    car.postEvent(forwardEvent);
    expect_process_radio_no_event();
    expect_move_forward(INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    car.processNextEvent();
}

TEST_F(MovementController_CarInit, BackwardEvent)
{
    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post BWD event. */
    car.postEvent(backwardEvent);
    expect_process_radio_no_event();
    expect_move_backward(INITIAL_MOTOR_SPEED, INITIAL_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    car.processNextEvent();
}

TEST_F(MovementController_CarInit, DISABLED_TurnRightEvent)
{
    uint32_t diff = 0;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post TURN RIGHT event. */
    car.postEvent(rightEvent);
    expect_process_radio_no_event();
    expect_turn_right(INITIAL_MOTOR_SPEED, TURN_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor right.*/
    expect_process_radio_no_event();
    expect_stop_motor_right();
    execute_irq(ctx.movementController->getRightTargetPulses());
    car.processNextEvent();
    /* Expect stop motor left.*/
    diff = ctx.movementController->getLeftTargetPulses() -
           ctx.movementController->getRightTargetPulses();
    expect_process_radio_no_event();
    expect_both_motors_stop();
    execute_irq(diff);
    car.processNextEvent();
}

TEST_F(MovementController_CarInit, DISABLED_TurnLeftEvent)
{
    uint32_t diff = 0;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post TURN LEFT event. */
    car.postEvent(leftEvent);
    expect_process_radio_no_event();
    expect_turn_left(TURN_MOTOR_SPEED, INITIAL_MOTOR_SPEED);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor left.*/
    expect_process_radio_no_event();
    expect_stop_motor_left();
    execute_irq(ctx.movementController->getLeftTargetPulses());
    car.processNextEvent();
    /* Expect stop motor right.*/
    diff = ctx.movementController->getRightTargetPulses() -
           ctx.movementController->getLeftTargetPulses();
    expect_process_radio_no_event();
    expect_both_motors_stop();
    execute_irq(diff);
    car.processNextEvent();
}

TEST_F(MovementController_CarInit, StopEvent)
{
    uint32_t target_pulses = 0;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post STOP event. */
    car.postEvent(stopEvent);
    expect_process_radio_no_event();
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &idleState);
}

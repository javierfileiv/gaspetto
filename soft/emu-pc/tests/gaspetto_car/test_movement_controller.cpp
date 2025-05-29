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
    expect_move_forward(MOTOR_FREQ, MOTOR_FREQ);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(ctx.movementController->getLeftTargetPulses());
    car.processNextEvent();
}

TEST_F(MovementController_CarInit, BackwardEvent)
{
    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post BWD event. */
    car.postEvent(backwardEvent);
    expect_move_backward(MOTOR_FREQ, MOTOR_FREQ);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(ctx.movementController->getLeftTargetPulses());
    car.processNextEvent();
}

TEST_F(MovementController_CarInit, TurnRightEvent)
{
    uint32_t diff = 0;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post TURN RIGHT event. */
    car.postEvent(rightEvent);
    expect_turn_right(MOTOR_FREQ, MOTOR_FREQ);
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
}

TEST_F(MovementController_CarInit, TurnLeftEvent)
{
    uint32_t diff = 0;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post TURN LEFT event. */
    car.postEvent(leftEvent);
    expect_turn_left(MOTOR_FREQ, MOTOR_FREQ);
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
}

TEST_F(MovementController_CarInit, StopEvent)
{
    uint32_t target_pulses = 0;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post STOP event. */
    car.postEvent(stopEvent);
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &idleState);
}

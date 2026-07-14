#include "fixture.h"

#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class MovementControllerTest : public Fixture {};

TEST_F(MovementControllerTest, InitOnly)
{
    ASSERT_EQ(get_low_power_mode(), true);
}

class MovementController_CarInit : public MovementControllerTest {};

TEST_F(MovementController_CarInit, ForwardEvent)
{
    /* Post FWD event. */
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    gaspettoCar.postEvent(forwardEvent);
    set_exit_low_power_mode();
    expect_process_radio_no_event();
    expect_update_movement();
    expect_move_forward(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    expect_update_movement();
    expect_target_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(MovementController_CarInit, BackwardEvent)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    /* Post BWD event. */
    gaspettoCar.postEvent(backwardEvent);
    set_exit_low_power_mode();
    expect_process_radio_no_event();
    expect_update_movement();
    expect_move_backward(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    expect_update_movement();
    expect_target_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(MovementController_CarInit, TurnRightEvent)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    /* Post TURN RIGHT event. */
    gaspettoCar.postEvent(rightEvent);
    set_exit_low_power_mode();
    expect_process_radio_no_event();
    expect_update_movement();
    expect_turn_right(90.0f, TURN_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    expect_update_movement();
    expect_target_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(MovementController_CarInit, TurnLeftEvent)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    /* Post TURN LEFT event. */
    gaspettoCar.postEvent(leftEvent);
    set_exit_low_power_mode();
    expect_process_radio_no_event();
    expect_update_movement();
    expect_turn_left(-90.0f, TURN_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
    expect_process_radio_no_event();
    expect_update_movement();
    expect_target_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(MovementController_CarInit, StopEvent)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    /* Post STOP event. */
    gaspettoCar.postEvent(stopEvent);
    set_exit_low_power_mode();
    expect_process_radio_no_event();
    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

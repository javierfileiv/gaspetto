#include "CommandPacket.h"
#include "fixture.h"

#include <cstring>
#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class RadioControllerTest : public Fixture {};

TEST_F(RadioControllerTest, InitOnly)
{
    ASSERT_EQ(get_low_power_mode(), true);
}

class RadioControllerTest_CarInit : public RadioControllerTest {};

TEST_F(RadioControllerTest_CarInit, ForwardEvent)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    /* Radio receive FWD event. */
    radio_receive_event(&forwardEvent);
    set_exit_low_power_mode();
    /* Motor moves FWD. */
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

TEST_F(RadioControllerTest_CarInit, BackwardEvent)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    /* Radio receive BWD event. */
    radio_receive_event(&backwardEvent);
    /* Motor moves BWD. */
    set_exit_low_power_mode();
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

TEST_F(RadioControllerTest_CarInit, TurnLeftEvent)
{
    /* Radio receive TURN LEFT event. */
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    radio_receive_event(&leftEvent);
    /* Motor TURNS LEFT. */
    set_exit_low_power_mode();
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

TEST_F(RadioControllerTest_CarInit, TurnRightEvent)
{
    /* Radio receive TURN RIGHT event. */
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    radio_receive_event(&rightEvent);
    /* Motor TURNS RIGHT. */
    set_exit_low_power_mode();
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

TEST_F(RadioControllerTest_CarInit, StopEvent)
{
    /* Radio receive STOP event. */
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    radio_receive_event(&stopEvent);
    /* Motor receives STOP. */
    set_exit_low_power_mode();
    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

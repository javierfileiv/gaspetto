#include "CommandPacket.h"
#include "fixture.h"

#include <gtest/gtest.h>

class QueueClearTest : public Fixture {};

TEST_F(QueueClearTest, IdleStateQueueClearReceived)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);

    radio_receive_event(&queueClearEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();

    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(QueueClearTest, ProcessingStateQueueClearReceived)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);

    radio_receive_event(&forwardEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_move_forward(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);

    radio_receive_event(&queueClearEvent);
    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();

    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(QueueClearTest, QueueClearDropsPendingCommands)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);

    /* Start moving first. */
    radio_receive_event(&forwardEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_move_forward(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);

    /* QUEUE_CLEAR must clear the command that follows in the same packet. */
    CommandPacket clearThenTurn{};
    clearThenTurn.count = 2;
    clearThenTurn.commands[0] = static_cast<uint8_t>(CommandId::QUEUE_CLEAR);
    clearThenTurn.commands[1] = static_cast<uint8_t>(CommandId::MOTOR_TURN_LEFT);
    radio_receive_raw(&clearThenTurn, sizeof(clearThenTurn));

    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();

    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(QueueClearTest, BackwardMovementQueueClearReceived)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);

    radio_receive_event(&backwardEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_move_backward(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);

    radio_receive_event(&queueClearEvent);
    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();

    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(QueueClearTest, TurnMovementQueueClearReceived)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);

    radio_receive_event(&rightEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_turn_right(90.0f, TURN_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);

    radio_receive_event(&queueClearEvent);
    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();

    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    ASSERT_EQ(get_low_power_mode(), true);
}

TEST_F(QueueClearTest, QueueClearThenNewCommandStillWorks)
{
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);

    radio_receive_event(&queueClearEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_stop_both_motors();
    expect_enter_low_power_mode();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);

    radio_receive_event(&forwardEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_move_forward(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();

    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
}

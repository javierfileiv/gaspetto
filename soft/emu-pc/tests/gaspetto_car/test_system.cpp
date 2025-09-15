#include "fixture.h"

#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class CarSystemBehaviorTest : public Fixture {};

TEST_F(CarSystemBehaviorTest, InitOnly)
{
}

TEST_F(CarSystemBehaviorTest, ForwardEvent)
{
    uint32_t current_pulses_left = 0;
    uint32_t current_pulses_right = 0;
    uint32_t target_pulses_left = 0;
    uint32_t target_pulses_right = 0;

    ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    /* Radio receive FWD event. */
    expect_receive_event(forwardEvent);
    RadioRxEvent(forwardEvent);
    /* Only one command, but pulses will be read several times by PID. */
    expect_read_irq_pulses();
    /* Motor moves FWD. */
    expect_move_forward(INITIAL_MOTOR_FREQ, INITIAL_MOTOR_FREQ);
    SimulateCarMainLoop([&]() { return gaspettoCar.getCurrentState() == &idleState; });
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
    /* No packet received in between. May be called several times in the loop. */
    expect_motors_set_speed();
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    SimulateCarMainLoop([&]() { return (gaspettoCar.getCurrentState() == &processingState); });
    /* Receive nothing. */
    RadioNoEventReceived();
}

// TEST_F(CarSystemBehaviorTest, BackwardEvent)
// {
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
//     /* Radio receive BWD event. */
//     expect_receive_event(backwardEvent);
//     ProcessRadio();
//     /* Motor moves BWD. */
//     expect_move_backward(INITIAL_MOTOR_FREQ, INITIAL_MOTOR_FREQ);
//     ProcessCar();
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
//     expect_stop_motor_left();
//     expect_stop_motor_right();
//     execute_irq(ctx.movementController->getLeftTargetPulses());
//     ProcessCar();
//     /* Receive nothing. */
//     RadioNoEventReceived();
// }

// TEST_F(CarSystemBehaviorTest, TurnRightEvent)
// {
//     uint32_t diff = 0;

//     /* Radio receive TURN RIGHT event. */
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
//     expect_receive_event(rightEvent);
//     ProcessRadio();
//     /* Motor TURNS RIGHT. */
//     expect_turn_right(INITIAL_MOTOR_FREQ, INITIAL_MOTOR_FREQ);
//     ProcessCar();
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
//     /* Expect stop motor right.*/
//     expect_stop_motor_right();
//     execute_irq(ctx.movementController->getRightTargetPulses());
//     ProcessCar();
//     /* Expect stop motor left.*/
//     diff = ctx.movementController->getLeftTargetPulses() -
//            ctx.movementController->getRightTargetPulses();
//     expect_both_motors_stop();
//     execute_irq(diff);
//     ProcessCar();
//     /* Receive nothing. */
//     RadioNoEventReceived();
// }

// TEST_F(CarSystemBehaviorTest, TurnLeftEvent)
// {
//     uint32_t diff = 0;

//     /* Radio receive TURN LEFT event. */
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
//     expect_receive_event(leftEvent);
//     ProcessRadio();
//     /* Motor TURNS LEFT. */
//     expect_turn_left(INITIAL_MOTOR_FREQ, INITIAL_MOTOR_FREQ);
//     ProcessCar();
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);
//     /* Expect stop motor left.*/
//     expect_stop_motor_left();
//     execute_irq(ctx.movementController->getLeftTargetPulses());
//     ProcessCar();
//     /* Expect stop motor right.*/
//     diff = ctx.movementController->getRightTargetPulses() -
//            ctx.movementController->getLeftTargetPulses();
//     expect_both_motors_stop();
//     execute_irq(diff);
//     ProcessCar();
//     /* Receive nothing. */
//     RadioNoEventReceived();
// }

// TEST_F(CarSystemBehaviorTest, StopEvent)
// {
//     /* Radio receive STOP event. */
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
//     expect_receive_event(stopEvent);
//     ProcessRadio();
//     /* Motor receives STOP. */
//     expect_both_motors_stop();
//     expect_enter_low_power_mode();
//     ProcessCar();
//     ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
//     /* Receive nothing. */
//     RadioNoEventReceived();
// }

#include "fixture.h"

#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class MotorControllerTest : public Fixture {
protected:
    void SetUp() override
    {
        Fixture::SetUp();
        expect_car_init();
        car.Init();
        ASSERT_EQ(car.getCurrentState(), &idleState);
    }
    void TearDown() override
    {
    }

public:
};

TEST_F(MotorControllerTest, InitOnly)
{
}

class MotorControllerNoRadioTest_CarInit : public MotorControllerTest {};

TEST_F(MotorControllerNoRadioTest_CarInit, ForwardEvent)
{
    uint8_t target_pulses = 0;
    const uint8_t distance = DISTANCE_CM_FWD_BWD;

    /* Post FWD event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    car.postEvent(forwardEvent);
    expect_move_forward(&target_pulses, distance, &target_pulses, distance);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(target_pulses);
    car.processNextEvent();
}

TEST_F(MotorControllerNoRadioTest_CarInit, BackwardEvent)
{
    uint8_t target_pulses = 0;
    const uint8_t distance = DISTANCE_CM_FWD_BWD;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post BWD event. */
    car.postEvent(backwardEvent);
    expect_move_backward(&target_pulses, distance, &target_pulses, distance);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(target_pulses);
    car.processNextEvent();
}

TEST_F(MotorControllerNoRadioTest_CarInit, TurnRightEvent)
{
    uint8_t target_pulses_left = 0, target_pulses_right = 0;
    uint8_t diff = 0;
    const uint8_t distance_left = DISTANCE_CM_FWD_BWD, distance_right = DISTANCE_CM_TURN_RIGHT;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post TURN RIGHT event. */
    car.postEvent(rightEvent);
    expect_turn_right(&target_pulses_left, distance_left, &target_pulses_right, distance_right);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor right.*/
    expect_stop_motor_right();
    execute_irq(target_pulses_right);
    car.processNextEvent();
    /* Expect stop motor left.*/
    diff = target_pulses_left - target_pulses_right;
    expect_both_motors_stop();
    execute_irq(diff);
    car.processNextEvent();
}

TEST_F(MotorControllerNoRadioTest_CarInit, TurnLeftEvent)
{
    uint8_t target_pulses_left = 0, target_pulses_right = 0;
    uint8_t diff = 0;
    const uint8_t distance_left = DISTANCE_CM_TURN_LEFT, distance_right = DISTANCE_CM_FWD_BWD;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post TURN LEFT event. */
    car.postEvent(leftEvent);
    expect_turn_left(&target_pulses_left, distance_left, &target_pulses_right, distance_right);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    /* Expect stop motor left.*/
    expect_stop_motor_left();
    execute_irq(target_pulses_left);
    car.processNextEvent();
    /* Expect stop motor right.*/
    diff = target_pulses_right - target_pulses_left;
    expect_both_motors_stop();
    execute_irq(diff);
    car.processNextEvent();
}

TEST_F(MotorControllerNoRadioTest_CarInit, StopEvent)
{
    uint8_t target_pulses = 0;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Post STOP event. */
    car.postEvent(stopEvent);
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &idleState);
}

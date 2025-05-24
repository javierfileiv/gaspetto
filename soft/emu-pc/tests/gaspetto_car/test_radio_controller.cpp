#include "fixture.h"

#include <gtest/gtest.h>

using ::testing::_;
using ::testing::Return;
using ::testing::StrictMock;

class RadioControllerTest : public Fixture {
protected:
    void SetUp() override
    {
        Fixture::SetUp();
        expect_car_init();
        car.Init();
        ASSERT_EQ(car.getCurrentState(), &idleState);
    }

    void postEventToTransmit(EventId eventId, CommandId commandId)
    {
        Event evt(eventId, commandId);
        mock_RadioController.SendEvent(evt);
    }

    void TearDown() override
    {
    }

public:
};

TEST_F(RadioControllerTest, InitOnly)
{
}

class RadioControllerWithMotorControllerTest : public RadioControllerTest {};

TEST_F(RadioControllerWithMotorControllerTest, ForwardEvent)
{
    uint8_t target_pulses = 0;
    const uint8_t distance = DISTANCE_CM_FWD_BWD;

    /* Radio receive FWD event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    mock_RadioController.expect_process_event(&forwardEvent);
    mock_RadioController.ProcessRadio();
    /* Motor moves FWD. */
    expect_move_forward(&target_pulses, distance, &target_pulses, distance);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(target_pulses);
    car.processNextEvent();
    mock_RadioController.expect_process_event(nullptr);
    mock_RadioController.ProcessRadio();
}

TEST_F(RadioControllerWithMotorControllerTest, BackwardEvent)
{
    uint8_t target_pulses = 0;
    const uint8_t distance = DISTANCE_CM_FWD_BWD;

    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Radio receive BWD event. */
    mock_RadioController.expect_process_event(&backwardEvent);
    mock_RadioController.ProcessRadio();
    /* Motor moves BWD. */
    expect_move_backward(&target_pulses, distance, &target_pulses, distance);
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &processingState);
    expect_stop_motor_left();
    expect_stop_motor_right();
    execute_irq(target_pulses);
    car.processNextEvent();
    /* Receive nothing. */
    mock_RadioController.expect_process_event(nullptr);
    mock_RadioController.ProcessRadio();
}

TEST_F(RadioControllerWithMotorControllerTest, TurnRightEvent)
{
    uint8_t target_pulses_left = 0, target_pulses_right = 0;
    uint8_t diff = 0;
    const uint8_t distance_left = DISTANCE_CM_FWD_BWD, distance_right = DISTANCE_CM_TURN_RIGHT;

    /* Radio receive TURN RIGHT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    mock_RadioController.expect_process_event(&rightEvent);
    mock_RadioController.ProcessRadio();
    /* Motor TURNS RIGHT. */
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
    /* Receive nothing. */
    mock_RadioController.expect_process_event(nullptr);
    mock_RadioController.ProcessRadio();
}

TEST_F(RadioControllerWithMotorControllerTest, TurnLeftEvent)
{
    uint8_t target_pulses_left = 0, target_pulses_right = 0;
    uint8_t diff = 0;
    const uint8_t distance_left = DISTANCE_CM_TURN_LEFT, distance_right = DISTANCE_CM_FWD_BWD;

    /* Radio receive TURN LEFT event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    mock_RadioController.expect_process_event(&leftEvent);
    mock_RadioController.ProcessRadio();
    /* Motor TURNS LEFT. */
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
    /* Receive nothing. */
    mock_RadioController.expect_process_event(nullptr);
    mock_RadioController.ProcessRadio();
}

TEST_F(RadioControllerWithMotorControllerTest, StopEvent)
{
    /* Radio receive STOP event. */
    ASSERT_EQ(car.getCurrentState(), &idleState);
    mock_RadioController.expect_process_event(&stopEvent);
    mock_RadioController.ProcessRadio();
    /* Motor receives STOP. */
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    car.processNextEvent();
    ASSERT_EQ(car.getCurrentState(), &idleState);
    /* Receive nothing. */
    mock_RadioController.expect_process_event(nullptr);
    mock_RadioController.ProcessRadio();
}

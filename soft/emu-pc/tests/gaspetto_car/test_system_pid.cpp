/**
 * @file test_system_pid.cpp
 *
 * System-level tests: radio event → GaspettoCar state machine →
 * real MovementController (PID) → mock IMU / mock MotorControl.
 *
 * Unlike test_pid_movement.cpp (which tests MovementController in isolation),
 * these tests exercise the full pipeline including state transitions.
 *
 * Fixture helpers:
 *   advanceTime(ms)          – step simulated clock
 *   setYaw(deg)              – set mock IMU reading
 *   radioReceiveEvent(evt)   – inject an event via the radio path
 *   runWorkCycles(n)         – call gaspettoCar.work() n times, advancing
 *                              time by PID_PERIOD_MS each cycle
 */

#include "CarEvents.h"
#include "CarStates.h"
#include "Context.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "IdleState.h"
#include "MovementController.h"
#include "ProcessingState.h"
#include "RadioController.h"
#include "config_event.h"
#include "config_radio.h"
#include "mock_Arduino.h"
#include "mock_IMUOrientation.h"
#include "mock_MotorControl.h"
#include "mock_RF24.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::NiceMock;
using testing::Return;

static constexpr uint32_t PID_PERIOD_MS = 50;

static const uint8_t *sys_writing_addr = gaspetto_box_pipe_name;
static const uint8_t *sys_reading_addr = gaspetto_car_pipe_name;

/* ---------------------------------------------------------------------------
 * SystemPIDTest fixture
 * --------------------------------------------------------------------------- */
class SystemPIDTest : public ::testing::Test {
protected:
    /* Low-level mocks. */
    NiceMock<MockArduino> mockArduino;
    NiceMock<MockIMUOrientation> mockIMU;
    NiceMock<MockMotorControl> mockMotorControl;
    NiceMock<MockRF24> mockRF24;

    /* Real objects. */
    MovementController movementController{ mockMotorControl, mockIMU };
    EventQueue eventQueue;
    RadioController radioController{ mockRF24, &eventQueue, sys_writing_addr, sys_reading_addr };

    IdleState idleState;
    ProcessingState processingState;
    Context ctx{ &eventQueue, &movementController, &radioController, nullptr,
                 &idleState,  &processingState,    MOTOR_FREQ };
    GaspettoCar gaspettoCar{ ctx };

    /* Simulated time and IMU state. */
    unsigned long currentTimeMs = 0;
    float simulatedYaw = 0.0f;

    /* Last captured motor command. */
    uint32_t lastLeftPWM = 0;
    uint32_t lastRightPWM = 0;
    bool lastLeftFwd = true;
    bool lastRightFwd = true;

    void SetUp() override
    {
        /* Controlled clock. */
        ON_CALL(mockArduino, millis()).WillByDefault([this]() { return currentTimeMs; });

        /* IMU always starts OK. */
        ON_CALL(mockIMU, _begin(_, _)).WillByDefault(Return(true));
        ON_CALL(mockIMU, _yaw()).WillByDefault([this]() { return simulatedYaw; });

        /* Spy on motor commands. */
        ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
                .WillByDefault([this](uint32_t l, uint32_t r, bool lf, bool rf) {
                    lastLeftPWM = l;
                    lastRightPWM = r;
                    lastLeftFwd = lf;
                    lastRightFwd = rf;
                });

        /* Radio init stubs. */
        ON_CALL(mockRF24, _begin()).WillByDefault(Return(true));
        ON_CALL(mockRF24, _setDataRate(_)).WillByDefault(Return(true));

        /* Init the full system. */
        gaspettoCar.setLowPowerModeCallback([]() { SwitchToLowPowerMode(); });
        gaspettoCar.init(StateId::IDLE);
        ASSERT_EQ(gaspettoCar.getCurrentStateId(), StateId::IDLE);
    }

    /* ---- helpers ---- */

    void advanceTime(uint32_t ms)
    {
        currentTimeMs += ms;
    }
    void setYaw(float yaw)
    {
        simulatedYaw = yaw;
    }

    /** Inject an event directly into the event queue (bypasses RF24 mock). */
    void postEvent(Event evt)
    {
        gaspettoCar.postEvent(evt);
    }

    /**
     * Run N work() cycles, each advancing the clock by PID_PERIOD_MS.
     * Radio reports "no data available" every cycle.
     */
    void runWorkCycles(int n)
    {
        for (int i = 0; i < n; ++i) {
            /* Radio: nothing incoming. */
            ON_CALL(mockRF24, _available(_)).WillByDefault(Return(false));
            advanceTime(PID_PERIOD_MS);
            gaspettoCar.work();
        }
    }

    /** Convenience: check current state. */
    bool isInState(StateId s) const
    {
        return gaspettoCar.getCurrentStateId() == s;
    }

    /* Pre-built events. */
    Event forwardEvent{ EventId::ACTION, CommandId::MOTOR_FORWARD };
    Event backwardEvent{ EventId::ACTION, CommandId::MOTOR_BACKWARD };
    Event stopEvent{ EventId::ACTION, CommandId::MOTOR_STOP };
    Event leftEvent{ EventId::ACTION, CommandId::MOTOR_TURN_LEFT };
    Event rightEvent{ EventId::ACTION, CommandId::MOTOR_TURN_RIGHT };
};

/* ===========================================================================
 * Tests
 * =========================================================================== */

TEST_F(SystemPIDTest, InitStartsInIdleState)
{
    EXPECT_TRUE(isInState(StateId::IDLE));
    EXPECT_FALSE(movementController.isMoving());
}

/* ---- Forward ---- */

TEST_F(SystemPIDTest, ForwardCommand_TransitionsToProcessing)
{
    postEvent(forwardEvent);
    runWorkCycles(1);
    EXPECT_TRUE(isInState(StateId::PROCESSING));
    EXPECT_TRUE(movementController.isMoving());
}

TEST_F(SystemPIDTest, ForwardCommand_ReturnsToIdleAfterTimeout)
{
    setYaw(0.0f);
    postEvent(forwardEvent);

    /* Drive until timeout (5000 ms = 100 cycles at 50 ms each). */
    runWorkCycles(101);

    EXPECT_FALSE(movementController.isMoving());
    EXPECT_TRUE(isInState(StateId::IDLE));
}

TEST_F(SystemPIDTest, ForwardCommand_DrivesStraightWithNoDrift)
{
    setYaw(0.0f);
    postEvent(forwardEvent);
    runWorkCycles(5);

    /* With zero yaw error, motors should drive roughly symmetrically forward. */
    EXPECT_TRUE(lastLeftFwd);
    EXPECT_TRUE(lastRightFwd);
    EXPECT_NEAR(lastLeftPWM, lastRightPWM, 2);
    EXPECT_GT(lastLeftPWM, 0u);
}

TEST_F(SystemPIDTest, ForwardCommand_PIDCorrectsYawDrift)
{
    setYaw(0.0f);
    postEvent(forwardEvent);
    runWorkCycles(2);

    /* Introduce drift. */
    setYaw(15.0f);
    runWorkCycles(3);

    /* PID should produce asymmetric motor output. */
    EXPECT_NE(lastLeftPWM, lastRightPWM);
}

/* ---- Backward ---- */

TEST_F(SystemPIDTest, BackwardCommand_TransitionsToProcessing)
{
    setYaw(0.0f);
    postEvent(backwardEvent);
    runWorkCycles(1);
    EXPECT_TRUE(isInState(StateId::PROCESSING));
    EXPECT_TRUE(movementController.isMoving());
}

/* ---- Turn ---- */

TEST_F(SystemPIDTest, TurnRightCommand_TransitionsToProcessing)
{
    setYaw(0.0f);
    postEvent(rightEvent);
    runWorkCycles(1);
    EXPECT_TRUE(isInState(StateId::PROCESSING));
    EXPECT_TRUE(movementController.isMoving());
}

TEST_F(SystemPIDTest, TurnRightCommand_StopsWhenYawReachesTarget)
{
    setYaw(0.0f);
    postEvent(rightEvent);
    runWorkCycles(1); /* Start the turn. */

    /* Simulate the car gradually reaching 90°. */
    const float steps[] = { 20, 45, 70, 85, 89, 90, 90, 90, 90 };
    for (float yaw : steps) {
        setYaw(yaw);
        ON_CALL(mockRF24, _available(_)).WillByDefault(Return(false));
        advanceTime(PID_PERIOD_MS);
        gaspettoCar.work();
    }

    EXPECT_FALSE(movementController.isMoving());
    EXPECT_TRUE(isInState(StateId::IDLE));
}

TEST_F(SystemPIDTest, TurnLeftCommand_StopsWhenYawReachesTarget)
{
    setYaw(0.0f);
    postEvent(leftEvent);
    runWorkCycles(1);

    const float steps[] = { -20, -45, -70, -85, -89, -90, -90, -90, -90 };
    for (float yaw : steps) {
        setYaw(yaw);
        ON_CALL(mockRF24, _available(_)).WillByDefault(Return(false));
        advanceTime(PID_PERIOD_MS);
        gaspettoCar.work();
    }

    EXPECT_FALSE(movementController.isMoving());
    EXPECT_TRUE(isInState(StateId::IDLE));
}

/* ---- Stop ---- */

TEST_F(SystemPIDTest, StopCommand_WhileProcessing_TransitionsToIdle)
{
    setYaw(0.0f);
    postEvent(forwardEvent);
    runWorkCycles(3);
    ASSERT_TRUE(isInState(StateId::PROCESSING));

    /* Send STOP while driving. */
    postEvent(stopEvent);
    runWorkCycles(1);

    EXPECT_FALSE(movementController.isMoving());
    EXPECT_TRUE(isInState(StateId::IDLE));
}

/* ---- Sequence of commands ---- */

TEST_F(SystemPIDTest, ForwardThenTurnRight_FullSequence)
{
    /* 1) Forward drives straight, times out. */
    setYaw(0.0f);
    postEvent(forwardEvent);
    runWorkCycles(101);
    ASSERT_TRUE(isInState(StateId::IDLE));

    /* 2) Turn right, yaw reaches 90°. */
    setYaw(0.0f);
    postEvent(rightEvent);
    runWorkCycles(1);
    ASSERT_TRUE(isInState(StateId::PROCESSING));

    const float steps[] = { 30, 60, 85, 89, 90, 90, 90, 90 };
    for (float yaw : steps) {
        setYaw(yaw);
        ON_CALL(mockRF24, _available(_)).WillByDefault(Return(false));
        advanceTime(PID_PERIOD_MS);
        gaspettoCar.work();
    }
    EXPECT_TRUE(isInState(StateId::IDLE));
}

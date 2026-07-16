/**
 * @file test_pid_movement.cpp
 *
 * Tests the real MovementController with mock IMU and mock MotorControl,
 * simulating time progression so the PID loop actually runs.
 *
 * Fixture provides:
 *   - advanceTime(ms)  : step the simulated clock
 *   - setYaw(degrees)  : set what the mock IMU reports
 *   - runPIDCycles(n)  : advance n×50 ms and call updateMovement() each cycle
 *
 * Extend by adding more TEST_F cases to this file.
 */

#include "MovementController.h"
#include "mock_Arduino.h"
#include "mock_IMUOrientation.h"
#include "mock_MotorControl.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::AnyNumber;
using testing::NiceMock;
using testing::Return;

static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint32_t PID_PERIOD_MS = 50; /* PID sample time. */

namespace
{
int *gTelemetryCounter = nullptr;

void telemetryCounterCallback(const TelemetryPacket &)
{
    if (gTelemetryCounter != nullptr) {
        (*gTelemetryCounter)++;
    }
}
}

/* ---------------------------------------------------------------------------
 * Fixture: real MovementController + mock IMU/motors + controlled clock.
 * --------------------------------------------------------------------------- */
class PIDMovementTest : public ::testing::Test {
protected:
    /* Mocks MUST be declared before movementController (construction order). */
    NiceMock<MockArduino> mockArduino;
    NiceMock<MockIMUOrientation> mockIMU;
    NiceMock<MockMotorControl> mockMotorControl;
    MovementController movementController{ mockMotorControl, mockIMU };

    unsigned long currentTimeMs = 0;
    float simulatedYaw = 0.0f;

    void SetUp() override
    {
        /* Clock always returns our controlled value. */
        ON_CALL(mockArduino, millis()).WillByDefault([this]() { return currentTimeMs; });

        /* IMU. */
        ON_CALL(mockIMU, _begin(_, _)).WillByDefault(Return(true));
        ON_CALL(mockIMU, _yaw()).WillByDefault([this]() { return simulatedYaw; });

        movementController.init(PWM_FREQ);
    }

    /* Advance the simulated clock. */
    void advanceTime(uint32_t ms)
    {
        currentTimeMs += ms;
    }

    /* Set the yaw the mock IMU will report from now on. */
    void setYaw(float yaw)
    {
        simulatedYaw = yaw;
    }

    /* Run N PID cycles (each = PID_PERIOD_MS). */
    void runPIDCycles(int n)
    {
        for (int i = 0; i < n; ++i) {
            advanceTime(PID_PERIOD_MS);
            movementController.updateMovement();
        }
    }
};

TEST_F(PIDMovementTest, InitOnly)
{
    EXPECT_FALSE(movementController.isMoving());
}

TEST_F(PIDMovementTest, StraightDriving_StaysOnHeading)
{
    /* Start straight driving at speed 50, 5 s timeout. */
    setYaw(0.0f);
    movementController.startStraightDriving(50.0f, 5000);
    EXPECT_TRUE(movementController.isMoving());

    /* Simulate a few cycles with zero yaw drift. */
    runPIDCycles(5);

    /* Should still be moving (hasn't timed out). */
    EXPECT_TRUE(movementController.isMoving());
}

TEST_F(PIDMovementTest, StraightDriving_MotorsReceiveSymmetricPWM)
{
    setYaw(0.0f);
    movementController.startStraightDriving(50.0f, 5000);

    /* Capture the last setMotorSpeeds call. */
    uint32_t lastLeft = 0, lastRight = 0;
    ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
            .WillByDefault([&](uint32_t l, uint32_t r, bool, bool) {
                lastLeft = l;
                lastRight = r;
            });

    runPIDCycles(3);

    /* On a straight heading, left and right PWM should be close to equal. */
    EXPECT_NEAR(lastLeft, lastRight, 2);
    EXPECT_GT(lastLeft, 0u);
}

TEST_F(PIDMovementTest, StraightDriving_CorrectionOnYawDrift)
{
    setYaw(0.0f);
    movementController.startStraightDriving(50.0f, 5000);
    runPIDCycles(2);

    /* Introduce a 10° drift to the right. */
    setYaw(10.0f);

    uint32_t lastLeft = 0, lastRight = 0;
    ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
            .WillByDefault([&](uint32_t l, uint32_t r, bool, bool) {
                lastLeft = l;
                lastRight = r;
            });

    runPIDCycles(3);

    /* PID should push a differential: left != right. */
    EXPECT_NE(lastLeft, lastRight);
}

TEST_F(PIDMovementTest, TurnInPlace_StopsWhenTargetReached)
{
    setYaw(0.0f);
    movementController.startTurningInPlace(90.0f, 30.0f, 5000);
    EXPECT_TRUE(movementController.isMoving());

    /* Gradually approach target yaw over several cycles. */
    const float steps[] = { 20, 45, 70, 85, 89, 90, 90, 90, 90 };
    for (float yaw : steps) {
        setYaw(yaw);
        advanceTime(PID_PERIOD_MS);
        movementController.updateMovement();
    }

    /* After 4 consecutive cycles inside the 3° deadband, movement should stop. */
    EXPECT_FALSE(movementController.isMoving());
}

TEST_F(PIDMovementTest, TimedMovement_StopsAfterTimeout)
{
    setYaw(0.0f);
    movementController.startStraightDriving(50.0f, 500);
    EXPECT_TRUE(movementController.isMoving());

    /* Advance just under the timeout. */
    runPIDCycles(9); /* 9 × 50 = 450 ms */
    EXPECT_TRUE(movementController.isMoving());

    /* Cross the 500 ms boundary. */
    runPIDCycles(1); /* 500 ms total */
    EXPECT_FALSE(movementController.isMoving());
}

TEST_F(PIDMovementTest, TurnTargetNormalizationAbove180IsHandled)
{
    setYaw(0.0f);
    movementController.startTurningInPlace(270.0f, 30.0f, 5000); /* normalizes to -90 */
    EXPECT_TRUE(movementController.isMoving());

    const float steps[] = { -30, -60, -85, -89, -90, -90, -90, -90 };
    for (float yaw : steps) {
        setYaw(yaw);
        advanceTime(PID_PERIOD_MS);
        movementController.updateMovement();
    }

    EXPECT_FALSE(movementController.isMoving());
}

TEST_F(PIDMovementTest, TurnTargetNormalizationBelowMinus180IsHandled)
{
    setYaw(0.0f);
    movementController.startTurningInPlace(-270.0f, 30.0f, 5000); /* normalizes to +90 */
    EXPECT_TRUE(movementController.isMoving());

    const float steps[] = { 30, 60, 85, 89, 90, 90, 90, 90 };
    for (float yaw : steps) {
        setYaw(yaw);
        advanceTime(PID_PERIOD_MS);
        movementController.updateMovement();
    }

    EXPECT_FALSE(movementController.isMoving());
}

TEST_F(PIDMovementTest, StraightDrivingWrapsPositiveDiffAcrossBoundary)
{
    setYaw(170.0f);
    movementController.startStraightDriving(50.0f, 5000);

    /* target=170, current=-170 => diff=340 => wrap branch diff>180 */
    setYaw(-170.0f);

    uint32_t lastLeft = 0, lastRight = 0;
    ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
            .WillByDefault([&](uint32_t l, uint32_t r, bool, bool) {
                lastLeft = l;
                lastRight = r;
            });

    runPIDCycles(3);
    EXPECT_NE(lastLeft, lastRight);
}

TEST_F(PIDMovementTest, StraightDrivingWrapsNegativeDiffAcrossBoundary)
{
    setYaw(-170.0f);
    movementController.startStraightDriving(50.0f, 5000);

    /* target=-170, current=170 => diff=-340 => wrap branch diff<-180 */
    setYaw(170.0f);

    uint32_t lastLeft = 0, lastRight = 0;
    ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
            .WillByDefault([&](uint32_t l, uint32_t r, bool, bool) {
                lastLeft = l;
                lastRight = r;
            });

    runPIDCycles(3);
    EXPECT_NE(lastLeft, lastRight);
}

TEST_F(PIDMovementTest, ZeroDurationModesRemainActiveUntilExplicitStop)
{
    setYaw(0.0f);
    movementController.startStraightDriving(35.0f, 0);
    runPIDCycles(6);
    EXPECT_TRUE(movementController.isMoving());

    movementController.stopMovement();
    EXPECT_FALSE(movementController.isMoving());

    movementController.startTurningInPlace(90.0f, 25.0f, 0);
    runPIDCycles(4);
    EXPECT_TRUE(movementController.isMoving());
}

TEST_F(PIDMovementTest, TelemetryCallbackRunsAtConfiguredInterval)
{
    int sent = 0;
    gTelemetryCounter = &sent;
    movementController.setTelemetryCallback(telemetryCounterCallback);

    setYaw(0.0f);
    movementController.startStraightDriving(40.0f, 5000);

    /* 10 cycles x 50ms = 500ms -> one telemetry frame should be sent. */
    runPIDCycles(10);
    EXPECT_GE(sent, 1);

    gTelemetryCounter = nullptr;
}

TEST(MovementControllerInitBranchTest, InitFailsWhenImuBeginFailsHalts)
{
    NiceMock<MockArduino> mockArduino;
    NiceMock<MockIMUOrientation> mockIMU;
    NiceMock<MockMotorControl> mockMotorControl;
    MovementController movementController{ mockMotorControl, mockIMU };

    ON_CALL(mockIMU, _begin(_, _)).WillByDefault(Return(false));

    EXPECT_DEATH(movementController.init(PWM_FREQ), "Assertion.*imuOk");
}

TEST_F(PIDMovementTest, UpdateMovementReturnsEarlyWhenIdle)
{
    EXPECT_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _)).Times(0);
    movementController.updateMovement();
}

TEST_F(PIDMovementTest, TurningWithLargeErrorClampsMotorSpeeds)
{
    setYaw(179.0f);
    movementController.startTurningInPlace(0.0f, 1000.0f, 5000);

    uint32_t left = 0;
    uint32_t right = 0;

    ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
            .WillByDefault([&](uint32_t l, uint32_t r, bool lf, bool rf) {
                left = l;
                right = r;
            });

    runPIDCycles(3);

    EXPECT_LE(left, 255u);
    EXPECT_LE(right, 255u);
    EXPECT_TRUE(left == 255u || right == 255u);
}

TEST_F(PIDMovementTest, StraightDrivingLargePositiveAndNegativeDriftProducesOppositeCorrections)
{
    setYaw(0.0f);
    movementController.startStraightDriving(120.0f, 5000);

    uint32_t leftPos = 0, rightPos = 0;
    uint32_t leftNeg = 0, rightNeg = 0;

    ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
            .WillByDefault([&](uint32_t l, uint32_t r, bool, bool) {
                leftPos = l;
                rightPos = r;
            });

    setYaw(170.0f);
    runPIDCycles(3);

    ON_CALL(mockMotorControl, _setMotorSpeeds(_, _, _, _))
            .WillByDefault([&](uint32_t l, uint32_t r, bool, bool) {
                leftNeg = l;
                rightNeg = r;
            });

    setYaw(-170.0f);
    runPIDCycles(3);

    EXPECT_NE(leftPos, rightPos);
    EXPECT_NE(leftNeg, rightNeg);
    EXPECT_NE((leftPos > rightPos), (leftNeg > rightNeg));
}

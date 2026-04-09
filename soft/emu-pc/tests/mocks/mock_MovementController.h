#pragma once

#include "MovementControllerInterface.h"
#include "mock_base.h"

#include <gmock/gmock.h>

/**
 * @brief Mock for MovementControllerInterface.
 *
 * Allows testing GaspettoCar state machine without PID/motor internals.
 */
class MockMovementController : public MockBase<MockMovementController>,
                               public MovementControllerInterface {
public:
    MockMovementController();
    ~MockMovementController();

    MOCK_METHOD(void, init, (uint32_t pwm_freq), (override));
    MOCK_METHOD(void, startStraightDriving, (float speed, uint32_t duration_ms), (override));
    MOCK_METHOD(void, startTurningInPlace, (float target_yaw, float speed, uint32_t duration_ms),
                (override));
    MOCK_METHOD(void, updateMovement, (), (override));
    MOCK_METHOD(void, stopBothMotors, (), (override));
    MOCK_METHOD(bool, isMoving, (), (const, override));
};

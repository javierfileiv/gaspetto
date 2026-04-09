#pragma once

#include <cstdint>

/**
 * @brief Interface for movement controller.
 *
 * Abstracts the motor/IMU/PID control so that higher-level code
 * (GaspettoCar, states) can be tested with a mock implementation.
 */
class MovementControllerInterface {
public:
    virtual ~MovementControllerInterface() = default;

    /** @brief Initialize the movement controller. */
    virtual void init(uint32_t pwm_freq) = 0;

    /** @brief Start PID-controlled straight driving. */
    virtual void startStraightDriving(float speed, uint32_t duration_ms = 0) = 0;

    /** @brief Start PID-controlled turn in place. */
    virtual void startTurningInPlace(float final_yaw_angle, float speed,
                                     uint32_t duration_ms = 0) = 0;

    /** @brief Update PID control loop (call from main loop). */
    virtual void updateMovement() = 0;

    /** @brief Stop all motors. */
    virtual void stopBothMotors() = 0;

    /** @brief Check if currently performing PID-controlled movement. */
    virtual bool isMoving() const = 0;
};

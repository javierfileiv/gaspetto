#pragma once

#include "ActiveObject.h"
#include "Arduino.h"
#include "CarEvents.h"
#include "CarStates.h"
#include "Context.h"
#include "Log.h"

#ifndef ARDUINO
#include "Arduino_pins_pc.h"
#endif
#include <stdint.h>

const uint32_t MOTOR_FREQ = 20000; /* Set PWM frequency to 20KHz. */

const uint32_t MOTOR_LEFT_BWD = PB_14; /* Example PWM pin for motor left. D4 on salaea. */
const uint32_t MOTOR_LEFT_FWD = PB_15; /* Direction pin for motor left.  D5 on salaea. */
const uint32_t MOTOR_RIGHT_FWD = PB_11; /* PWM pin for motor right. D1 on salaea. */
const uint32_t MOTOR_RIGHT_BWD = PB_10; /* Direction pin for motor right. D2 on salaea. */

/** @brief Number of states for GaspettoCar. */
constexpr uint8_t CAR_MAX_STATES = static_cast<uint8_t>(StateId::MAX_STATE_ID);

const float INITIAL_MOTOR_SPEED = 50;
const uint32_t TURN_MOTOR_SPEED = 30;
const uint32_t MOTOR_TIMEOUT_MS = 5000; /* Default timeout for motor commands. */

/**
 * @brief Active object base type for GaspettoCar.
 */
using CarActiveObject = GenericActiveObject<StateId, Event, CAR_MAX_STATES>;

class GaspettoCar : public CarActiveObject {
public:
    /** GaspettoCar(): Constructor for the GaspettoCar class.
     *  @ctx: Reference to the Context instance containing dependencies.
     */
    GaspettoCar(Context &ctx);

    /** init(): Initialize the GaspettoCar instance.
     *  @initialStateId: The initial state ID to start the state machine.
     */
    void init(StateId initialStateId = StateId::IDLE);

    /** * startStraightDriving: begin straight driving with PID control
     * @speed: PWM speed value
     * @timeout_ms: optional duration in milliseconds to drive before stopping (0 for unlimited)
     */
    void setMotorStraightDrive(float speed, uint32_t timeout_ms);

    /** * startTurningInPlace: begin turning in place to a target yaw
     * @final_yaw_angle: target yaw angle in degrees
     * @speed: base speed for turning
     * @duration_ms: optional duration in milliseconds (0 for unlimited)
     */
    void setMotorTurnInPlace(float final_yaw_angle, float speed, uint32_t timeout_ms);

    /* stopBothMotors: stop all motor movement immediately */
    void stopBothMotors(void);

    /** clearEventQueue(): Clear all pending events from the queue.
     */
    void clearEventQueue(void);

    /** postEvent(): Post an event to the event queue.
     * @evt: The event to be posted.
     * @return 0 on success, -1 on failure.
     */
    int postEvent(Event evt) override;

    /** work(): Perform work for the active object.
     *  Delegates to the current state.
     */
    void work(void) override;

    /** enterLowPowerMode(): Enter low power mode.
     */
    void enterLowPowerMode(void) override;

    bool isTargetReached(void);

    /**
     * transitionTo(): Transition to a new state (expose base class method).
     * @param newStateId Target state.
     */
    using CarActiveObject::transitionTo;

private:
    /**
     * InitMotorPins(): Initialize the motor pins.
     * Sets the pin modes for the motor control pins.
     */
    void InitMotorPins();

private:
    Context &_ctx;
};

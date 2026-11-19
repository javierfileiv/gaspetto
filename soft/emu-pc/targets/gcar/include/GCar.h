#pragma once

#include "ActiveObject.h"
#include "Arduino.h"
#include "Context.h"
#include "Events.h"
#include "Log.h"
#include "States.h"

#include <pin_definitions.h>

#ifndef ARDUINO
#include "Arduino_pins_pc.h"
#endif
#include <stdint.h>

/* 15 Hz, and deliberately so: at 20 kHz the same duty cycle smooths into an
 * average voltage below the motor stiction threshold and the car does not move
 * at all. At this frequency each period delivers a full-voltage pulse that
 * actually starts the motor, and the duty then sets the speed. Raising the
 * frequency to slow the car down does the opposite, because it also raises the
 * smallest duty that can move it */
const uint32_t MOTOR_FREQ = 15;

/** @brief Number of states for GCar. */
constexpr uint8_t CAR_MAX_STATES = static_cast<uint8_t>(StateId::MAX_STATE_ID);

const float INITIAL_MOTOR_SPEED = 37;
/* Turns need more duty than the straight drive: the wheels scrub sideways.
 * Measured at 15 Hz, a steady 20 turned at about 10 deg/s and a saturated 70
 * at about 86 deg/s. The duty is now held at this value for the whole turn,
 * which that curve puts near 30 deg/s, so a quarter turn takes about three
 * seconds instead of a burst followed by a crawl */
const uint32_t TURN_MOTOR_SPEED = 34;
const uint32_t MOTOR_TIMEOUT_MS = 2500; /* Default timeout for motor commands. */
/* A quarter turn takes longer than a forward burst, and this timeout is only
 * the safety net: the turn ends on its angle, well before it */
const uint32_t TURN_MOTOR_TIMEOUT_MS = 4000;

/**
 * @brief Active object base type for GCar.
 */
using CarActiveObject = GenericActiveObject<StateId, Event, CAR_MAX_STATES>;

class GCar : public CarActiveObject {
public:
    /** GCar(): Constructor for the GCar class.
     *  @ctx: Reference to the Context instance containing dependencies.
     */
    GCar(Context &ctx);

    /** init(): Initialize the GCar instance.
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

    /** * setMotorTurnBy: turn by a delta from the current heading
     * @delta_yaw: signed angle in degrees, positive turns counter-clockwise
     * @speed: base speed for turning
     * @timeout_ms: optional duration in milliseconds (0 for unlimited)
     */
    void setMotorTurnBy(float delta_yaw, float speed, uint32_t timeout_ms);

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

    /** setDfuCallback(): Install the function that enters the DFU bootloader
     * @callback: called once the motors are stopped; null keeps the request inert
     */
    void setDfuCallback(void (*callback)(void));

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
    void (*dfuCallback_)(void) = nullptr;
};

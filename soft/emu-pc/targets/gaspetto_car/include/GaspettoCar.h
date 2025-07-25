#pragma once

#include "ActiveObject.h"
#include "Arduino.h"
#include "Context.h"
#include "Log.h"

#ifndef ARDUINO
#include "Arduino_pins_pc.h"
#endif
#include <stdint.h>

typedef void (*LowPowerModeCallback)();
const uint32_t MOTOR_FREQ = 20000; /* Set PWM frequency to 20KHz. */
const uint32_t INITIAL_MOTOR_SPEED = 40;
const uint32_t TURN_MOTOR_SPEED = 20;
const uint32_t MOTOR_LEFT_BWD = PB14; /* Example PWM pin for motor left. D4 on salaea. */
const uint32_t MOTOR_LEFT_FWD = PB15; /* Direction pin for motor left.  D5 on salaea. */
const uint32_t MOTOR_RIGHT_FWD = PB11; /* PWM pin for motor right. D1 on salaea. */
const uint32_t MOTOR_RIGHT_BWD = PB10; /* Direction pin for motor right. D2 on salaea. */
const uint32_t SPEED_SENSOR_LEFT_PIN = PA1; /* Pin for left speed/distance sensor. D0 on salaea. */
const uint32_t SPEED_SENSOR_RIGHT_PIN = PA0; /* Pin for right speed/distance sensor. D3 on salaea.*/
const uint32_t DISTANCE_CM_FWD_BWD = 50;
const uint32_t DISTANCE_CM_TURN_RIGHT = 15;
const uint32_t DISTANCE_CM_TURN_LEFT = 20;

class GaspettoCar : public ActiveObject {
public:
    /** GaspettoCar(): Constructor for the GaspettoCar class.
     *  @ctx: Reference to the Context instance containing dependencies.
     */
    GaspettoCar(Context &ctx);

    /** init(): Initialize the GaspettoCar instance.
     *  @initialStateId: The initial state ID to start the state machine.
     */
    void init(StateId initialStateId = StateId::IDLE);

    /** setMotor(): Set the motor speeds and directions.
     * @forward_motor_left: Direction for the left motor.
     * (true for forward, false for backward).
     * @motor_left_speed: Speed for the left motor.
     * @distance_cm_left: Distance in centimeters for the left motor.
     * @forward_motor_right: Direction for the right motor.
     * (true for forward, false for backward).
     * @motor_right_speed: Speed for the right motor.
     * @distance_cm_right: Distance in centimeters for the right motor.
     */
    void setMotor(bool forward_motor_left, uint32_t motor_left_speed, uint32_t distance_cm_left,
                  bool forward_motor_right, uint32_t motor_right_speed, uint32_t distance_cm_right);

    /** postEvent(): Post an event to the event queue.
     * @evt: The event to be posted.
     * @return 0 on success, -1 on failure.
     */
    int postEvent(Event evt) override;

    /** processNextEvent(): Processe the next event in the event queue.
     *  Delegates to the current state.
     */
    void processNextEvent() override;

    /**
     * setLowPowerModeCallback(): Get the MovementController instance from the context.
     * @cb: Pointer to the low power callback function.
     */
    void setLowPowerModeCallback(void (*cb)(void))
    {
        enter_low_power_mode = cb;
    }

    /** enterLowPowerMode(): Enter low power mode.
     */
    void enterLowPowerMode() override;

    /**
     *  stopMotorRight(): Stop the right motor.
     *  Delegates to MovementController in the context.
     */
    void stopMotorRight();

    /**
     *  stopMotorLeft(): Stop the left motor.
     *  Delegates to MovementController in the context.
     */
    void stopMotorLeft();

    /**
     * resetCounterMotorRight(): Stop both motors and resets the pulse counters.
     * Delegates to MovementController in the context.
     */
    void resetCounterMotorRight();

    /**
     * resetCounterMotorLeft(): Stop both motors and resets the pulse counters.
     * Delegates to MovementController in the context.
     */
    void resetCounterMotorLeft();

private:
    /**
     * InitMotorPins(): Initialize the motor pins.
     * Sets the pin modes for the motor control pins.
     */
    void InitMotorPins();

    /**
     * isTargetReached(): Check if the current movement command's target has been reached.
     * Delegates to MovementController in the context.
     * @return True if the target is reached or if in IDLE state, false otherwise.
     */
    bool isTargetReached();

private:
    Context &_ctx;
    void (*enter_low_power_mode)(void);
};

#include "GaspettoCar.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "CarEvents.h"
#include "Context.h"
#include "EventQueue.h"
#include "MovementControllerInterface.h"
#include "__assert.h"

#ifdef USE_RADIO_CONTROLLER
#include "RadioController.h"
#else
#include "TimeredEventQueue.h"
#endif

#include <cstdint>

class State;

GaspettoCar::GaspettoCar(Context &ctx)
        : CarActiveObject()
        , _ctx(ctx)
{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GaspettoCar::init(StateId initialStateId)
{
    G_ASSERT(_ctx.movementController);
    _ctx.movementController->init(_ctx.pwm_freq);
#ifdef USE_RADIO_CONTROLLER
    G_ASSERT(_ctx.radioController);
    _ctx.radioController->init();
#else
    G_ASSERT(_ctx.timeredEventQueue);
#endif

    CarActiveObject::init(initialStateId);
}

void GaspettoCar::setMotorStraightDrive(float speed, uint32_t timeout_ms)
{
    _ctx.movementController->startStraightDriving(speed, timeout_ms);
}

void GaspettoCar::setMotorTurnInPlace(float final_yaw_angle, float speed, uint32_t timeout_ms)
{
    _ctx.movementController->startTurningInPlace(final_yaw_angle, speed, timeout_ms);
}

void GaspettoCar::stopBothMotors(void)
{
    _ctx.movementController->stopBothMotors();
}

bool GaspettoCar::isTargetReached(void)
{
    return !_ctx.movementController->isMoving();
}

int GaspettoCar::postEvent(Event evt)
{
    if (_ctx.mainEventQueue) {
        _ctx.mainEventQueue->enqueue(evt);
        return 0;
    }
    return -1;
}

void GaspettoCar::work(void)
{
#ifdef USE_RADIO_CONTROLLER
    _ctx.radioController->processRadio();
#else
    _ctx.timeredEventQueue->process(*this);
#endif

    /* Update movement controller (PID, telemetry) - handles its own timing. */
    _ctx.movementController->updateMovement();

    if (!_ctx.mainEventQueue->IsEmpty()) {
        Event evt;

        StateType *currentState = states[static_cast<uint8_t>(currentStateIndex)];
        _ctx.mainEventQueue->dequeue(evt);
        currentState->processEvent(evt);
    }
    if (getCurrentStateId() != StateId::IDLE && isTargetReached()) {
        logln(F("Target reached. Transitioning to IDLE state."));
        transitionTo(StateId::IDLE);
    }
}

void GaspettoCar::enterLowPowerMode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    logln("Entering low-power mode...\n");
    if (lowPowerCallback_) {
        lowPowerCallback_();
    }
#else
    /*  Implement low-power mode for Arduino here. */
    /*  STM32 sleep modes or power-saving features. */
    delay(100); /*  Simulate low-power sleep. */
#endif
#endif
}

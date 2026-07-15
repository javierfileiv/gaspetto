#include "GCar.h"

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

GCar::GCar(Context &ctx)
        : CarActiveObject()
        , _ctx(ctx)
{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GCar::init(StateId initialStateId)
{
    logln(F("GCar: checking movement controller"));
    G_ASSERT(_ctx.movementController);
    logln(F("GCar: init movement controller"));
    _ctx.movementController->init(_ctx.pwm_freq);
#ifdef USE_RADIO_CONTROLLER
    logln(F("GCar: init radio controller"));
    G_ASSERT(_ctx.radioController);
    _ctx.radioController->init();
#else
    logln(F("GCar: timered event queue mode"));
    G_ASSERT(_ctx.timeredEventQueue);
#endif

    logln(F("GCar: init state machine"));
    CarActiveObject::init(initialStateId);
    logln(F("GCar: state machine ready"));
}

void GCar::setMotorStraightDrive(float speed, uint32_t timeout_ms)
{
    _ctx.movementController->startStraightDriving(speed, timeout_ms);
}

void GCar::setMotorTurnInPlace(float final_yaw_angle, float speed, uint32_t timeout_ms)
{
    _ctx.movementController->startTurningInPlace(final_yaw_angle, speed, timeout_ms);
}

void GCar::stopBothMotors(void)
{
    _ctx.movementController->stopBothMotors();
}

void GCar::clearEventQueue(void)
{
    if (_ctx.mainEventQueue) {
        _ctx.mainEventQueue->clear();
    }
}

bool GCar::isTargetReached(void)
{
    return !_ctx.movementController->isMoving();
}

int GCar::postEvent(Event evt)
{
    if (!_ctx.mainEventQueue) {
        logln(F("GCar RX: main event queue unavailable."));
        return -1;
    }

    log(F("GCar RX: "));
    log(eventIdToString(evt.getEventId()));
    if (evt.getEventId() == EventId::ACTION) {
        log(F(" "));
        log(commandIdToString(evt.getPayload()));
    }

    if (_ctx.mainEventQueue->IsFull()) {
        logln(F(" dropped; queue full."));
        return -1;
    }

    _ctx.mainEventQueue->enqueue(evt);
    log(F(" queued; queue size="));
    logln(static_cast<int>(_ctx.mainEventQueue->GetSize()));
    return 0;
}

void GCar::work(void)
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
        log(F("GCar RX: processing "));
        log(eventIdToString(evt.getEventId()));
        if (evt.getEventId() == EventId::ACTION) {
            log(F(" "));
            log(commandIdToString(evt.getPayload()));
        }
        logln();
        currentState->processEvent(evt);
    }
    if (getCurrentStateId() != StateId::IDLE && isTargetReached()) {
        logln(F("Target reached. Transitioning to IDLE state."));
        transitionTo(StateId::IDLE);
    }
}

void GCar::enterLowPowerMode()
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

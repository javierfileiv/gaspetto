#include "GCar.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "Context.h"
#include "EventQueue.h"
#include "Events.h"
#include "MovementControllerInterface.h"
#include "__assert.h"

#ifdef USE_RADIO_CONTROLLER
#include "RadioController.h"
#else
#include "TimeredEventQueue.h"
#endif

#include <cstdint>

class State;

namespace
{
/* The commands that have to wait for the movement already running. Everything
 * else, a stop, a queue clear or the bootloader, is an order that must never
 * wait, or a program could not be interrupted */
bool isMovementCommand(CommandId id)
{
    switch (id) {
    case CommandId::MOTOR_FORWARD:
    case CommandId::MOTOR_BACKWARD:
    case CommandId::MOTOR_TURN_LEFT:
    case CommandId::MOTOR_TURN_RIGHT:
        return true;
    default:
        return false;
    }
}
} /* namespace */

GCar::GCar(Context &ctx)
        : CarActiveObject()
        , _ctx(ctx)
{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GCar::init(StateId initialStateId)
{
#ifdef USE_RADIO_CONTROLLER
    /* The radio comes up before anything that can assert: a halted board
     * cannot receive the DFU request that would rescue it */
    LOGLN(F("GCar: init radio controller"));
    G_ASSERT(_ctx.radioController);
    _ctx.radioController->init();
#endif

    LOGLN(F("GCar: checking movement controller"));
    G_ASSERT(_ctx.movementController);
    LOGLN(F("GCar: init movement controller"));
    _ctx.movementController->init(_ctx.pwm_freq);
#ifndef USE_RADIO_CONTROLLER
    LOGLN(F("GCar: timered event queue mode"));
    G_ASSERT(_ctx.timeredEventQueue);
#endif

    LOGLN(F("GCar: init state machine"));
    CarActiveObject::init(initialStateId);
    LOGLN(F("GCar: state machine ready"));
}

void GCar::setMotorStraightDrive(float speed, uint32_t timeout_ms)
{
    _ctx.movementController->startStraightDriving(speed, timeout_ms);
}

void GCar::setMotorTurnInPlace(float final_yaw_angle, float speed, uint32_t timeout_ms)
{
    _ctx.movementController->startTurningInPlace(final_yaw_angle, speed, timeout_ms);
}

void GCar::setMotorTurnBy(float delta_yaw, float speed, uint32_t timeout_ms)
{
    _ctx.movementController->startTurningBy(delta_yaw, speed, timeout_ms);
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
        LOGLN(F("GCar RX: main event queue unavailable."));
        return -1;
    }

    LOG(F("GCar RX: "));
    LOG(eventIdToString(evt.getEventId()));
    if (evt.getEventId() == EventId::ACTION) {
        LOG(F(" "));
        LOG(commandIdToString(evt.getPayload()));
    }

    if (_ctx.mainEventQueue->IsFull()) {
        LOGLN(F(" dropped; queue full."));
        return -1;
    }

    _ctx.mainEventQueue->enqueue(evt);
    LOG(F(" queued; queue size="));
    LOGLN(static_cast<int>(_ctx.mainEventQueue->GetSize()));
    return 0;
}

void GCar::work(void)
{
#ifdef USE_RADIO_CONTROLLER
    _ctx.radioController->processRadio();
#else
    _ctx.timeredEventQueue->process(*this);
#endif

    if (!_ctx.mainEventQueue->IsEmpty()) {
        commandsQueued_ = true;
    }

    /* Update movement controller (PID, telemetry) - handles its own timing. */
    _ctx.movementController->updateMovement();

    if (!_ctx.mainEventQueue->IsEmpty()) {
        /* A program is a list of movements, so a queued one has to wait for the
         * one running. Taking it now would hand it to the state machine, which
         * refuses it, and that is how a four-command program ran only its
         * first command. The test is on the state, not on the movement: a
         * movement that just ended leaves the car in PROCESSING until the end
         * of this same call, and the next command would be refused there */
        const Event *next = _ctx.mainEventQueue->peek();
        const bool held = next != nullptr && next->getEventId() == EventId::ACTION &&
                          isMovementCommand(next->getPayload()) &&
                          getCurrentStateId() != StateId::IDLE;

        if (!held) {
            Event evt;

            StateType *currentState = states[static_cast<uint8_t>(currentStateIndex)];
            _ctx.mainEventQueue->dequeue(evt);
            LOG(F("GCar RX: processing "));
            LOG(eventIdToString(evt.getEventId()));
            if (evt.getEventId() == EventId::ACTION) {
                LOG(F(" "));
                LOG(commandIdToString(evt.getPayload()));
            }
            LOGLN();

            /* A DFU request is a device command, not a state behaviour: it has
             * to work from IDLE and from PROCESSING alike, so the state machine
             * never sees it. The motors are stopped first because the boot ROM
             * leaves the bridge pins exactly as the application left them */
            if (evt.getEventId() == EventId::DFU_REQUEST) {
                LOGLN(F("GCar: DFU request, stopping the motors"));
                stopBothMotors();
                if (dfuCallback_ != nullptr) {
                    dfuCallback_();
                }
                return;
            }

            currentState->processEvent(evt);
        }
    }

    if (getCurrentStateId() != StateId::IDLE && isTargetReached()) {
        LOGLN(F("Target reached. Transitioning to IDLE state."));
        transitionTo(StateId::IDLE);
    }

    /* A batch is over when the queue has drained and the car is back in IDLE.
     * Report it once: the box can animate the real end, not the transmission */
    if (commandsQueued_ && _ctx.mainEventQueue->IsEmpty() && getCurrentStateId() == StateId::IDLE) {
        commandsQueued_ = false;
        if (programDoneCallback_ != nullptr) {
            LOGLN(F("GCar: batch finished, reporting it"));
            programDoneCallback_();
        }
    }
}

void GCar::setDfuCallback(void (*callback)(void))
{
    dfuCallback_ = callback;
}

void GCar::setProgramDoneCallback(void (*callback)(void))
{
    programDoneCallback_ = callback;
}

void GCar::enterLowPowerMode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    LOGLN("Entering low-power mode...\n");
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

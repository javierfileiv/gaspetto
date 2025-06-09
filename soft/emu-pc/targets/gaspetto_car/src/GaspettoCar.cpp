#include "GaspettoCar.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "Context.h"
#include "MovementController.h"
#include "RadioController.h"
#include "TimeredEventQueue.h"

#include <cstdint>

class State;

GaspettoCar::GaspettoCar(Context &ctx)
        : _ctx(ctx)
        , eventQueue(EVENT_QUEUE_SIZE)
        , ActiveObject()

{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GaspettoCar::init(StateId initialStateId)
{
    _ctx.movementController->init(_ctx.pwm_freq);
    _ctx.radioController->setEventQueue(&eventQueue);
    _ctx.radioController->init();
    ActiveObject::init(initialStateId);
}

void GaspettoCar::setMotor(bool forward_motor_left, uint32_t motor_left_speed,
                           uint32_t distance_cm_left, bool forward_motor_right,
                           uint32_t motor_right_speed, uint32_t distance_cm_right)
{
    if (_ctx.movementController) {
        _ctx.movementController->setMotor(forward_motor_left, motor_left_speed, distance_cm_left,
                                          forward_motor_right, motor_right_speed,
                                          distance_cm_right);
    }
}

bool GaspettoCar::isTargetReached()
{
    bool right_reached = _ctx.movementController->getRightPulseCount() >
                         _ctx.movementController->getRightTargetPulses();
    bool left_reached = _ctx.movementController->getLeftPulseCount() >
                        _ctx.movementController->getLeftTargetPulses();

    if (currentStateId == StateId::IDLE)
        return false;
    if (left_reached)
        _ctx.movementController->stopMotorLeft();
    if (right_reached)
        _ctx.movementController->stopMotorRight();
    if (right_reached && left_reached) {
        _ctx.movementController->resetCounterMotorRight();
        _ctx.movementController->resetCounterMotorLeft();
        return true;
    }
    return false;
}

int GaspettoCar::postEvent(Event evt)
{
    if (!eventQueue.IsFull()) {
        eventQueue.enqueue(evt);
        return 0;
    }
    return -1;
}

bool GaspettoCar::postRadioEvent(TelemetryData evt)
{
    if (_ctx.radioController)
        return _ctx.radioController->getRadioQueue()->enqueue(evt);
    return false;
}

void GaspettoCar::stopMotorRight()
{
    _ctx.movementController->stopMotorRight();
}

void GaspettoCar::stopMotorLeft()
{
    _ctx.movementController->stopMotorLeft();
}

void GaspettoCar::processNextEvent()
{
    if (_ctx.radioController)
        _ctx.radioController->processRadio();
    if (_ctx.timeredEventQueue)
        _ctx.timeredEventQueue->processEvents(*this);
    if (!eventQueue.IsEmpty()) {
        Event evt;

        State *currentState = states[static_cast<uint8_t>(currentStateId)];
        eventQueue.dequeue(evt);
        currentState->processEvent(evt);
    }
    if (isTargetReached())
        transitionTo(StateId::IDLE);
}

void GaspettoCar::enterLowPowerMode()
{
    logln(F("IdleState: low power mode."));
    if (enter_low_power_mode) {
        enter_low_power_mode();
        return;
    }
}

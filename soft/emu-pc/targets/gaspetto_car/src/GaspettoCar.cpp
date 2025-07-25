#include "GaspettoCar.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "Context.h"
#include "MovementController.h"
#include "RadioController.h"

#include <cstdint>

class State;

GaspettoCar::GaspettoCar(Context &ctx)
        : _ctx(ctx)
        , ActiveObject(ctx.mainEventQueue, ctx.timeredEventQueue)

{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GaspettoCar::init(StateId initialStateId)
{
    _ctx.movementController->init(_ctx.pwm_freq);
#ifdef USE_RADIO_CONTROLLER
    _ctx.radioController->setEventQueue(&eventQueue);
    _ctx.radioController->init();
#endif
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
    bool right_reached = _ctx.movementController->getRightPulseCount() >=
                         _ctx.movementController->getRightTargetPulses();
    bool left_reached = _ctx.movementController->getLeftPulseCount() >=
                        _ctx.movementController->getLeftTargetPulses();

    if (currentStateId == StateId::IDLE)
        return true;
    if (left_reached)
        _ctx.movementController->stopMotorLeft();
    if (right_reached)
        _ctx.movementController->stopMotorRight();
    if (right_reached && left_reached)
        return true;
    return false;
}

int GaspettoCar::postEvent(Event evt)
{
    if (eventQueue) {
        eventQueue->enqueue(evt);
        return 0;
    }
    return -1;
}

void GaspettoCar::stopMotorRight()
{
    logln(F("GaspettoCar::stopMotorRight() - Delegating to MovementController."));
    _ctx.movementController->stopMotorRight();
}

void GaspettoCar::stopMotorLeft()
{
    logln(F("GaspettoCar::stopMotorLeft() - Delegating to MovementController."));
    _ctx.movementController->stopMotorLeft();
}

void GaspettoCar::processNextEvent()
{
#ifdef USE_RADIO_CONTROLLER
    if (_ctx.radioController) {
        _ctx.radioController->processRadio();
    }
#else
    if (_ctx.timeredEventQueue) {
        _ctx.timeredEventQueue->processEvents(*this);
    }
#endif
    // if (isTargetReached()) {
    if (eventQueue && !eventQueue->IsEmpty()) {
        Event evt;

        State *currentState = states[static_cast<uint8_t>(currentStateId)];
        eventQueue->dequeue(evt);
        currentState->processEvent(evt);
    }
    // }
}

void GaspettoCar::enterLowPowerMode()
{
    if (enter_low_power_mode) {
        enter_low_power_mode();
        return;
    }
}

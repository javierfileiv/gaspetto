#include "GaspettoBox.h"

#include "ActiveObject.h"
#include "Arduino.h"
#include "CarEvents.h"
#include "Context.h"
#include "RadioController.h"

#include <cstdint>

GaspettoBox::GaspettoBox(Context &ctx)
        : GenericActiveObject()
        , _ctx(ctx)
{
    initMachine(StateId::IDLE, ctx.idleState);
    initMachine(StateId::PROCESSING, ctx.processingState);
}

void GaspettoBox::init(StateId initialStateId)
{
    GenericActiveObject::init(initialStateId);
}

int GaspettoBox::postEvent(Event evt)
{
    if (_ctx.mainEventQueue) {
        _ctx.mainEventQueue->enqueue(evt);
        return 0;
    }
    return -1;
}

void GaspettoBox::work()
{
    if (_ctx.mainEventQueue && !_ctx.mainEventQueue->IsEmpty()) {
        Event evt;

        StateType *currentState = states[static_cast<uint8_t>(currentStateIndex)];
        _ctx.mainEventQueue->dequeue(evt);
        currentState->processEvent(evt);
    }
}

void GaspettoBox::enterLowPowerMode()
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

void GaspettoBox::debounceAndEnqueue(Event &evt, unsigned long currentTime)
{
#ifdef ARDUINO
    if (currentTime - lastDebounceTime > debounceDelay) {
        lastDebounceTime = currentTime;
        if (!_ctx.mainEventQueue->IsFull()) {
            _ctx.mainEventQueue->enqueue(evt);
            logln("Exiting low-power mode...\n");
            lowPowerMode = false; /*  Wake the system up. */
        } else {
            logln("Event queue is full! Unable to enqueue event.\n");
        }
    }
#else
    if (!_ctx.mainEventQueue->IsFull()) {
        postEvent(evt);
    } else {
        logln("Event queue is full! Unable to enqueue event.\n");
    }
#endif
}

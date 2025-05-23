#include "GaspettoBox.h"

#include <iostream>

GaspettoBox::GaspettoBox(State *idle, State *running, EventQueue *queue)
        : ActiveObject(queue, nullptr)
{
    _initialStateId = StateId::IDLE;
    InitMachine(StateId::IDLE, idle);
    InitMachine(StateId::PROCESSING, running);
}

void GaspettoBox::Init()
{
    ActiveObject::Init();
}

int GaspettoBox::postEvent(Event evt)
{
    if (eventQueue) {
        eventQueue->enqueue(evt);
        return 0;
    }
    return -1;
}

void GaspettoBox::processNextEvent()
{
    if (eventQueue && !eventQueue->IsEmpty()) {
        Event evt;

        State *currentState = states[static_cast<uint8_t>(currentStateId)];
        eventQueue->dequeue(evt);
        currentState->processEvent(evt);
    }
}

void GaspettoBox::enterLowPowerMode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    Serial.println("Entering low-power mode...\n");
    SwitchToLowPowerMode();
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
        if (!eventQueue->full()) {
            eventQueue->enqueue(evt);
            Serial.println("Exiting low-power mode...\n");
            lowPowerMode = false; /*  Wake the system up. */
        } else {
            Serial.println("Event queue is full! Unable to enqueue event.\n");
        }
    }
#else
    if (!eventQueue->IsFull()) {
        postEvent(evt);
    } else {
        Serial.println("Event queue is full! Unable to enqueue event.\n");
    }
#endif
}

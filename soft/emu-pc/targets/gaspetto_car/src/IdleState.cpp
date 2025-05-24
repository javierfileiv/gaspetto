#include "IdleState.h"

#include "GaspettoCar.h"

#include <cstdint>

#ifndef ARDUINO
#include <cassert>
#include <iostream>
#endif

void IdleState::enter()
{
    active_object->enterLowPowerMode();
}

void IdleState::processEvent(Event &evt)
{
    active_object->transitionTo(StateId::PROCESSING);
    State *currentState = active_object->getCurrentState();
    currentState->processEvent(evt);
}

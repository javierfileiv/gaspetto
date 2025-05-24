#include "IdleState.h"

#include "GaspettoBox.h"

#include <cassert>
#include <iostream>

void IdleState::enter()
{
    active_object->enterLowPowerMode();
}

void IdleState::processEvent(Event &evt)
{
    switch (evt.getEventId()) {
    case EventId::BUTTON_PRESSED:
        active_object->transitionTo(StateId::PROCESSING);
        break;
    default:
        break;
    }
}

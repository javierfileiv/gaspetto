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
    Serial.println("Processing event in IdleState...\n");
    switch (evt.getEventId()) {
    case EventId::BUTTON_PRESSED:
        active_object->transitionTo(StateId::PROCESSING);
        break;
    default:
        /* Stay in low power mode*/
        // state_machine->enterLowPowerMode();
        break;
    }
}

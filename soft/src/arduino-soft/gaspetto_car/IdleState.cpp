#include "IdleState.h"

#include "GaspettoCar.h"

#include <cassert>
#include <iostream>

void IdleState::enter()
{
    state_machine->enterLowPowerMode();
}

void IdleState::processEvent(Event event)
{
    Serial.println("Processing event in IdleState...\n");
    switch (event.getEventId()) {
    case EventId::NRF_IRQ:
        state_machine->transitionTo(StateId::PROCESSING);
        state_machine->processEvent(event);
        break;
    default:
        /* Stay in low power mode. */
        state_machine->enterLowPowerMode();
        break;
    }
}

#include "IdleState.h"
#include "GaspettoBox.h"
#include <cassert>
#include <iostream>

void IdleState::enter() { state_machine->enterLowPowerMode(); }

void IdleState::processEvent(Event event) {
  Serial.println("Processing event in IdleState...\n");
  switch (event.getEventId()) {
  case EventId::BUTTON_PRESSED:
    state_machine->transitionTo(StateId::PROCESSING);
    break;
  default:
    /* Stay in low power mode*/
    state_machine->enterLowPowerMode();
    break;
  }
}
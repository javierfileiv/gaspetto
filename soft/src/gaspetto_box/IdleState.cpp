#include "IdleState.h"
#include "GaspettoBox.h"
#include <cassert>
#include <iostream>

void IdleState::enter() { box_state_machine->enterLowPowerMode(); }

void IdleState::processEvent(Event event) {
  std::cout << "Processing event in IdleState...\n";
  switch (event.getEventId()) {
  case EventId::BUTTON_PRESSED:
    box_state_machine->transitionTo(StateId::PROCESSING);
    break;
  default:
    /* Stay in low power mode*/
    box_state_machine->enterLowPowerMode();
    break;
  }
}
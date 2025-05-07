#include "IdleState.h"
#include "GaspettoCar.h"
#include <cassert>
#include <iostream>

void IdleState::enter() {
  car_state_machine->enterLowPowerMode();
}

void IdleState::processEvent(Event event) {
  std::cout << "Processing event in IdleState...\n";
  switch (event.getEventId()) {
  case EventId::NRF_IRQ:
    car_state_machine->postEvent(event);
    car_state_machine->transitionTo(StateId::PROCESSING);
    break;
  default:
    assert(true);
    break;
  }
}
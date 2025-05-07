#include "State.h"
#include "IdleState.h"
#include "iostream"

void IdleState::enter() {
  std::cout << "Entering Idle State...\n";
}
void IdleState::exit() {
  std::cout << "Exiting Idle State...\n";
}
void IdleState::processEvent(Event event) {
  std::cout << "Processing event in Idle State...\n";
}
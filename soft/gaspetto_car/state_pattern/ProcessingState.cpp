#include "ProcessingState.h"
#include "EventQueue.h"
#include <iostream>

void ProcessingState::enter() {
  std::cout << "Entering Idle State...\n";
}
void ProcessingState::exit() {
  std::cout << "Exiting Idle State...\n";
}
void ProcessingState::processEvent(EventData event) {
  std::cout << "Processing event in Idle State...\n";
}
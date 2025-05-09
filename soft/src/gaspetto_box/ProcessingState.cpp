#include "ProcessingState.h"
#include "GaspettoBox.h"
#include "StateMachine.h"
#include <cassert>
#include <iostream>

void ProcessingState::enter() {
  for (int row = 0; row < 3; ++row) {
    std::cout << "Processing row " << row << "...\n";
    std::cout << "Scanning row " << row << "...\n";
    std::this_thread::sleep_for(
        std::chrono::duration<int>(1)); // Simulate some processing work
    std::cout << "Sending row " << row << "...\n";
  }
  box_state_machine->transitionTo(StateId::IDLE);
}
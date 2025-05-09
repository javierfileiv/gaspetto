#pragma once

#include "StateMachine.h"
#include <iostream>
#define MAX_EVENTS 16
class GaspettoBox;
class Event;

class State {
public:
  virtual void enter() {
    std::cout << "Default enter method in State class\n";
  }
  virtual void exit() {
    std::cout << "Default exit method in State class\n";
  }
  virtual void processEvent(Event evt) {
    std::cout << "Default processEvent method in State class\n";
  }
  void setMachine(GaspettoBox *m) { box_state_machine = m; }

protected:
  GaspettoBox *box_state_machine;
};
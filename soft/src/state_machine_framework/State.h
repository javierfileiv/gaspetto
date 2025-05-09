#pragma once

#include "Arduino.h"
#include "Event.h"
#include <iostream>
#define MAX_EVENTS 16
class ActiveObject;
class Event;

class State {
public:
  virtual void enter() {
    Serial.println("Default enter method in State class\n");
  }
  virtual void exit() {
    Serial.println("Default exit method in State class\n");
  }
  virtual void processEvent(Event evt) {
    Serial.println("Default processEvent method in State class\n");
  }
  void setMachine(ActiveObject *m) { state_machine = m; }

protected:
  ActiveObject *state_machine;
};
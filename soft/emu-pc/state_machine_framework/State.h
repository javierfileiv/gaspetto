#ifndef STATE_H
#define STATE_H

#include "Arduino.h"
#include "Event.h"
#ifndef ARDUINO
#include <iostream>
#endif

#define MAX_EVENTS 16
class ActiveObject;
class Event;

class State {
public:
    virtual void enter()
    {
        Serial.println("Default enter method in State class\n");
    }
    virtual void exit()
    {
        Serial.println("Default exit method in State class\n");
    }
    virtual void processEvent(Event evt)
    {
        Serial.println("Default processEvent method in State class\n");
    }
    void setMachine(ActiveObject *m)
    {
        state_machine = m;
    }

protected:
    ActiveObject *state_machine;
};
#endif /* STATE_H */

#ifndef STATE_H
#define STATE_H

#include "Arduino.h"
#ifndef ARDUINO
#include <iostream>
#endif

class Event;
class ActiveObject;

enum class StateId : uint8_t { IDLE, PROCESSING, PAUSED, MAX_STATE_ID };
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
    virtual void processEvent(Event &evt)
    {
        Serial.println("Default processEvent method in State class\n");
    }
    void setMachine(ActiveObject *m)
    {
        active_object = m;
    }

protected:
    ActiveObject *active_object;
};
#endif /* STATE_H */

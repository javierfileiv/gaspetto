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
    }
    virtual void exit()
    {
    }
    virtual void processEvent(Event &evt)
    {
    }
    void setMachine(ActiveObject *m)
    {
        active_object = m;
    }

protected:
    ActiveObject *active_object;
};
#endif /* STATE_H */

#ifndef ACTIVE_OBJECT_H
#define ACTIVE_OBJECT_H

#include "Arduino.h"
#include "EventQueue.h"
#include "State.h"
#include "TimeredEventQueue.h" // Ensure this header file exists and defines TimeredEventQueue

#ifndef ARDUINO
#include <cstdint>
#include <iostream>
#include <thread>
#endif

class TimeredEventQueue;
class ActiveObject {
public:
    ActiveObject(EventQueue *queue, TimeredEventQueue *timeredQueue)
            : eventQueue(queue)
            , timeredEventQueue(timeredQueue)
            , states{}
    {
    }

    void InitMachine(StateId state_id, State *state)
    {
        /* Initialize state. */
        states[static_cast<uint8_t>(state_id)] = state;
        /* Set up state machine references. */
        if (states[static_cast<uint8_t>(state_id)] != nullptr)
            states[static_cast<uint8_t>(state_id)]->setMachine(this);
    }

    void Init()
    {
        currentStateId = _initialStateId;
        states[static_cast<int>(currentStateId)]->enter();
    }

    void transitionTo(StateId newStateId)
    {
        State *currentState = states[static_cast<int>(currentStateId)];

        currentState->exit();
        currentStateId = newStateId;
        currentState = states[static_cast<uint8_t>(currentStateId)];
        currentState->enter();
    }

    State *getCurrentState()
    {
        return states[static_cast<uint8_t>(currentStateId)];
    }

    virtual int postEvent(Event evt) = 0;

    virtual void processNextEvent() = 0;

    virtual void enterLowPowerMode() = 0;

    EventQueue *getEventQueue()
    {
        return eventQueue;
    }

    TimeredEventQueue *getTimeredEventQueue()
    {
        return timeredEventQueue;
    }

protected:
    EventQueue *eventQueue;
    TimeredEventQueue *timeredEventQueue;
    State *states[static_cast<uint8_t>(StateId::MAX_STATE_ID)];
    StateId currentStateId;
    StateId _initialStateId;
};

#endif // ACTIVE_OBJECT_H

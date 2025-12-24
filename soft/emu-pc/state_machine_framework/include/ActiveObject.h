#ifndef ACTIVE_OBJECT_H
#define ACTIVE_OBJECT_H

#include "Arduino.h"
#include "State.h"

#ifndef ARDUINO
#include <cstdint>
#endif

class EventQueue;
class TimeredEventQueue;
class Log;

class ActiveObject : public Log {
public:
    virtual ~ActiveObject() = default;

    ActiveObject(EventQueue *queue, TimeredEventQueue *timeredQueue)
            : eventQueue(queue)
            , timeredEventQueue(timeredQueue)
            , states{}
            , currentStateId(StateId::IDLE)
    {
    }

    void initMachine(StateId state_id, State *state)
    {
        /* Initialize state. */
        states[static_cast<uint8_t>(state_id)] = state;
        /* Set up state machine references. */
        if (states[static_cast<uint8_t>(state_id)] != nullptr)
            states[static_cast<uint8_t>(state_id)]->setMachine(this);
    }

    void init(StateId initialStateId)
    {
        currentStateId = initialStateId;
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
};

#endif /* ACTIVE_OBJECT_H. */

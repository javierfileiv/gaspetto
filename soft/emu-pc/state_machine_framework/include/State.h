#ifndef STATE_H
#define STATE_H

#include "Arduino.h"
#include "Log.h"

class ActiveObjectBase;

/**
 * @brief Abstract base class for FSM states.
 *
 * Implements the State pattern. Concrete states override enter(), exit(),
 * and processEvent() to define state-specific behavior.
 *
 * @tparam EventT The event type used by the state machine.
 */
template <typename EventT> class GenericState : public Log {
public:
    virtual ~GenericState() = default;

    /**
     * @brief Called when entering this state.
     */
    virtual void enter()
    {
    }

    /**
     * @brief Called when exiting this state.
     */
    virtual void exit()
    {
    }

    /**
     * @brief Handle an event in this state.
     * @param evt The event to process.
     */
    virtual void processEvent(EventT &evt)
    {
        (void)evt;
    }

    /**
     * @brief Set the owning state machine.
     * @param machine The ActiveObject that owns this state.
     */
    void setMachine(ActiveObjectBase *machine)
    {
        active_object_ = machine;
    }

protected:
    ActiveObjectBase *active_object_ = nullptr;
};

#endif /* STATE_H */

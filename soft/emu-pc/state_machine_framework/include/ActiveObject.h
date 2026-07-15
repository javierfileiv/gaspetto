#ifndef ACTIVE_OBJECT_H
#define ACTIVE_OBJECT_H

#include "Arduino.h"
#include "Log.h"
#include "State.h"

#ifndef ARDUINO
#include <stdint.h>
#endif

/** @brief Callback type for low power mode. */
typedef void (*LowPowerModeCallback)();

/**
 * @brief Non-templated base class for ActiveObject.
 *
 * Provides the interface that State classes can use without
 * needing to know the specific StateId/Event types.
 */
class ActiveObjectBase : public Log {
public:
    virtual ~ActiveObjectBase() = default;

    /** @brief Enter low power mode. */
    virtual void enterLowPowerMode() = 0;

    /**
     * @brief Set the low power mode callback.
     * @param callback Function to call for low power mode.
     */
    void setLowPowerModeCallback(LowPowerModeCallback callback)
    {
        lowPowerCallback_ = callback;
    }

protected:
    LowPowerModeCallback lowPowerCallback_ = nullptr;
};

/**
 * @brief Abstract base class for active objects (event-driven state machines).
 *
 * Implements the Active Object pattern with a hierarchical state machine.
 * Derived classes must implement postEvent(), work(), and
 * enterLowPowerMode().
 *
 * @tparam StateIdT  Enum type for state identifiers.
 * @tparam EventT    Event type for the state machine.
 * @tparam MaxStates Maximum number of states (typically derived from StateIdT::MAX_STATE_ID).
 */
template <typename StateIdT, typename EventT, uint8_t MaxStates>
class GenericActiveObject : public ActiveObjectBase {
public:
    using StateType = GenericState<EventT>;

    /**
     * @brief Construct a GenericActiveObject.
     */
    GenericActiveObject()
            : states{}
            , currentStateIndex(static_cast<StateIdT>(0))
    {
    }

    virtual ~GenericActiveObject() = default;

    /**
     * @brief Register a state with the state machine.
     * @param stateId The state identifier.
     * @param state Pointer to the state object.
     */
    void initMachine(StateIdT stateId, StateType *state)
    {
        const auto idx = static_cast<uint8_t>(stateId);
        if (idx >= MaxStates) {
            return;
        }
        states[idx] = state;
        if (state != nullptr) {
            state->setMachine(this);
        }
    }

    /**
     * @brief Initialize and enter the initial state.
     * @param initialStateId Starting state.
     */
    void init(StateIdT initialStateId)
    {
        currentStateIndex = initialStateId;
        StateType *initial = getCurrentState();
        if (initial != nullptr) {
            initial->enter();
        }
    }

    /**
     * @brief Transition to a new state.
     * @param newStateId Target state.
     */
    void transitionTo(StateIdT newStateId)
    {
        StateType *current = getCurrentState();
        if (current != nullptr) {
            current->exit();
        }

        currentStateIndex = newStateId;

        StateType *next = getCurrentState();
        if (next != nullptr) {
            next->enter();
        }
    }

    /**
     * @brief Get the current state object.
     * @return Pointer to current state, or nullptr.
     */
    StateType *getCurrentState() const
    {
        const auto idx = static_cast<uint8_t>(currentStateIndex);
        if (idx >= MaxStates) {
            return nullptr;
        }
        return states[idx];
    }

    /**
     * @brief Get the current state ID.
     * @return Current StateIdT.
     */
    StateIdT getCurrentStateId() const
    {
        return currentStateIndex;
    }

    /** @brief Post an event to the queue. */
    virtual int postEvent(EventT evt) = 0;

    /** @brief Work for active object. */
    virtual void work() = 0;

protected:
    StateType *states[MaxStates];
    StateIdT currentStateIndex;
};

#endif /* ACTIVE_OBJECT_H */

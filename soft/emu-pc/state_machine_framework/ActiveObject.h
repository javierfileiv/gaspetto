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
    {
    }

    void InitMachine(StateId state_id, State *state)
    {
        /* Initialize state. */
        states[static_cast<int>(state_id)] = state;
        /* Set up state machine references. */
        for (int i = 0; i < static_cast<int>(StateId::MAX_STATE_ID); i++) {
            if (states[i] != nullptr) {
                states[i]->setMachine(this);
            }
        }
    }
    void SetInitialState(StateId state)
    {
        currentStateId = state;
    }

    void Init(void)
    {
        states[static_cast<int>(currentStateId)]->enter();
    }

    void transitionTo(StateId newStateId)
    {
        State *currentState = states[static_cast<int>(currentStateId)];

        currentState->exit();
        currentStateId = newStateId;
        currentState = states[static_cast<int>(currentStateId)];
        currentState->enter();
    }

    int postEvent(Event evt)
    {
        if (eventQueue->IsFull()) {
            Serial.println("Event queue is full, cannot post event.\n");
            return -1;
        }
        if (eventQueue) {
#ifdef DEBUG_GASPETTO
            Serial.print("postEvent(): ");
            Serial.print(reinterpret_cast<const char *>(
                    event_to_string[static_cast<int>(evt.getEventId())].str));
            Serial.print(", ");
            Serial.print("Command: ");
            Serial.print(reinterpret_cast<const char *>(
                    command_to_string[static_cast<int>(evt.getCommand())].str));
            Serial.print(", ");
            Serial.print("Current Time: ");
            Serial.print(millis());
            Serial.println(" ms");
#endif
            eventQueue->enqueue(evt);
            return 0;
        }
        return -1;
    }

    virtual void processNextEvent(void)
    {
        if (eventQueue && !eventQueue->IsEmpty()) {
            Event evt;

            State *currentState = states[static_cast<int>(currentStateId)];
            eventQueue->dequeue(evt);
            currentState->processEvent(evt);
        }
    }

    void processEvent(Event evt)
    {
        State *currentState = states[static_cast<int>(currentStateId)];
#ifdef DEBUG_GASPETTO
        Serial.print("processEvent: ");
        Serial.print(reinterpret_cast<const char *>(
                event_to_string[static_cast<int>(evt.getEventId())].str));
        Serial.print(", ");
        Serial.print("Command: ");
        Serial.print(reinterpret_cast<const char *>(
                command_to_string[static_cast<int>(evt.getCommand())].str));
        Serial.print(", ");
        Serial.print("Current Time: ");
        Serial.print(millis());
        Serial.println(" ms");
#endif
        currentState->processEvent(evt);
    }

    virtual void enterLowPowerMode(void)
    {
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
        Serial.println("Entering low-power mode...\n");
        lowPowerMode = true;
        while (lowPowerMode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); /*  Simulate
                                                                            low-power
                                                                            sleep. */
        }
#else
        /*  Implement low-power mode for Arduino here. */
        /*  For example, you might use sleep modes or power-saving features. */
        /*  specific to the Arduino platform.. */
        delay(100); /*  Simulate low-power sleep. */
#endif
#endif
    }

protected:
    EventQueue *eventQueue;
    TimeredEventQueue *timeredEventQueue;
    State *states[static_cast<int>(StateId::MAX_STATE_ID)];
    StateId currentStateId;
};

#endif // ACTIVE_OBJECT_H

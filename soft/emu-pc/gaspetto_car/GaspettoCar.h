#pragma once

#include "ActiveObject.h"
#include "Arduino.h"
#include "EventQueue.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "State.h"

#ifndef ARDUINO
#include <cstdint>
#define __ASSERT_USE_STDERR
#include <assert.h>
#else
#define assert(x)                              \
    do {                                       \
        Serial.print(F("Assertion failed: ")); \
        Serial.print(F(#x));                   \
        Serial.print(F(" in "));               \
        Serial.print(__FILE__);                \
        Serial.print(F(" at line "));          \
        Serial.println(__LINE__);              \
        while (1) {                            \
            delay(1000);                       \
        }                                      \
    } while (0)
#endif
class GaspettoCar : public ActiveObject {
public:
    GaspettoCar(State *idle, State *running, EventQueue *queue, StateId initial_state);

    void enqueue_random_commands(const uint8_t num_events);
};

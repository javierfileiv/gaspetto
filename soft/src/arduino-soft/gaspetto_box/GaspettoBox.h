#include "ActiveObject.h"
#include "Arduino.h"
#include "EventQueue.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "State.h"

#include <cstdint>
#define __ASSERT_USE_STDERR
#include <assert.h>

#ifndef ARDUINO
extern std::atomic<bool> lowPowerMode;
#endif

class GaspettoBox : public ActiveObject {
public:
    GaspettoBox(State *idle, State *running, EventQueue *queue, StateId initial_state);

    void debounceAndEnqueue(Event &evt, unsigned long currentTime);
};

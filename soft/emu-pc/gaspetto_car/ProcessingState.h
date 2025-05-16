#pragma once

#include "Event.h"
#include "State.h"

class Event;

#define FORWARD true
#define BACKWARD false

class ProcessingState : public State {
public:
    void processEvent(Event evt) override;
};

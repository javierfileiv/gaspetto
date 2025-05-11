#pragma once

#include "Event.h"
#include "State.h"

class Event;

class ProcessingState : public State {
public:
    void enter() override;
};

#pragma once
#include "State.h"

class Event;

class IdleState : public State {
public:
    void enter() override;
    void processEvent(Event &evt) override;
};

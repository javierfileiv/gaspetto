#pragma once
#include "Event.h"
#include "State.h"

class IdleState : public State {
public:
    void enter() override;
    void processEvent(Event evt) override;
};

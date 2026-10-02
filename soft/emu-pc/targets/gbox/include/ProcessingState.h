#pragma once

#include "Events.h"
#include "State.h"

class ProcessingState : public GenericState<Event> {
public:
    void enter() override;
    void processEvent(Event &evt) override;
};

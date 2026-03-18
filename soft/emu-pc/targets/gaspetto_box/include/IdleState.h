#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "CarEvents.h"
#include "State.h"

class IdleState : public GenericState<Event> {
public:
    void enter() override;
    void processEvent(Event &evt) override;
};
#endif /* IDLE_STATE_H */

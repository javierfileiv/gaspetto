#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "Event.h"
#include "State.h"

class IdleState : public State {
public:
    void enter() override;
    void processEvent(Event &evt) override;
};
#endif /* IDLE_STATE_H */

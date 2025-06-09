#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "State.h"

class Event;
class IdleState : public State {
public:
    void enter() override;
    void processEvent(Event &evt) override;
};
#endif /* IDLE_STATE_H */

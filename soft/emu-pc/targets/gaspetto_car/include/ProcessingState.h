#ifndef PROCESSING_STATE_H
#define PROCESSING_STATE_H

#include "Event.h"
#include "State.h"

class Event;

class ProcessingState : public State {
public:
    void processEvent(Event &evt) override;
};
#endif /* PROCESSING_STATE_H */

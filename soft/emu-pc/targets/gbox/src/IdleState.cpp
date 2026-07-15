#include "IdleState.h"

#include "GBox.h"

void IdleState::enter()
{
    active_object_->enterLowPowerMode();
}

void IdleState::processEvent(Event &evt)
{
    GBox *box = static_cast<GBox *>(active_object_);

    switch (evt.getEventId()) {
    case EventId::BUTTON_PRESSED:
        box->transitionTo(StateId::PROCESSING);
        break;
    default:
        break;
    }
}

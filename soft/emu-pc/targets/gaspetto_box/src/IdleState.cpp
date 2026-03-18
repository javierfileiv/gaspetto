#include "IdleState.h"

#include "GaspettoBox.h"

void IdleState::enter()
{
    active_object_->enterLowPowerMode();
}

void IdleState::processEvent(Event &evt)
{
    GaspettoBox *box = static_cast<GaspettoBox *>(active_object_);

    switch (evt.getEventId()) {
    case EventId::BUTTON_PRESSED:
        box->transitionTo(StateId::PROCESSING);
        break;
    default:
        break;
    }
}

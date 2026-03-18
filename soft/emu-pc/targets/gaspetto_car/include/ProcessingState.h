#pragma once

#include "CarEvents.h"
#include "State.h"

/**
 * @brief Processing state for GaspettoCar.
 *
 * Handles motor commands and transitions back to idle on stop.
 */
class ProcessingState : public GenericState<Event> {
public:
    void processEvent(Event &evt) override;
};

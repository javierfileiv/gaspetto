#pragma once

#include "CarEvents.h"
#include "State.h"

/**
 * @brief Idle state for GaspettoCar.
 *
 * Enters low power mode and waits for motor commands.
 */
class IdleState : public GenericState<Event> {
public:
    void enter() override;
    void processEvent(Event &evt) override;
};

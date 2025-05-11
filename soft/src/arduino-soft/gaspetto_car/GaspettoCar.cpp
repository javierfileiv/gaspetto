#include "GaspettoCar.h"

#include "ActiveObject.h"
#include "State.h"

#include <iostream>

GaspettoCar::GaspettoCar(State *idle, State *running, EventQueue *queue, StateId initial_state)
        : ActiveObject(queue, nullptr)
{
    InitMachine(StateId::IDLE, idle);
    InitMachine(StateId::PROCESSING, running);
    SetInitialState(initial_state);
}

void GaspettoCar::enqueue_random_commands(const uint8_t num_events)
{
    std::srand(time(nullptr));

    for (uint8_t i = 0; i < num_events; ++i) {
        Event event = { EventId::NRF_IRQ, static_cast<CommandId>(rand() % 4) }; /*  Random
                                                                                   event. */

        postEvent(event);
#ifdef LOW_POWER_MODE
        lowPowerMode = false; /*  Wake up the system. */
#endif
    }
}

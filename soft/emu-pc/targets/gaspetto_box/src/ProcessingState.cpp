#include "ProcessingState.h"

#include "GaspettoBox.h"
#include "State.h"

void ProcessingState::enter()
{
    for (int row = 0; row < 3; ++row) {
        log("Scanning row ");
        log(row);
        logln("...\n");
#ifndef ARDUINO
        std::this_thread::sleep_for(std::chrono::duration<int>(1)); /*  Simulate some
                                                                       processing work. */
#else
        delay(1000); /*  Simulate some processing work. */
#endif
        log("Processing row ");
        log(row);
        logln("...\n");
#ifndef ARDUINO
        std::this_thread::sleep_for(std::chrono::duration<int>(1)); /*  Simulate some
                                                                       processing work. */
#else
        delay(1000); /*  Simulate some processing work. */
#endif
        log("Sending row ");
        log(row);
        logln("...\n");
#ifndef ARDUINO
        std::this_thread::sleep_for(std::chrono::duration<int>(1)); /*  Simulate some
                                                                       processing work. */
#else
        delay(1000); /*  Simulate some processing work. */
#endif
    }
    active_object->transitionTo(StateId::IDLE);
}

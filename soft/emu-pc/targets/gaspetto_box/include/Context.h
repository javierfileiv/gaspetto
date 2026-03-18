#ifndef CONTEXT_H
#define CONTEXT_H

#include "CarEvents.h"
#include "EventQueue.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "TimeredEventQueue.h"

class RadioController;

using EventQueue = GenericEventQueue<Event>;
using TimeredEventQueue = GenericTimeredEventQueue<Event>;

/**
 * A struct to encapsulate and manage the core dependencies and services
 * of the robot system. This centralizes the "global" objects, making
 * them easier to pass around and manage.
 */
struct Context {
    EventQueue *mainEventQueue;
    TimeredEventQueue *timeredEventQueue;
    RadioController *radioController;
    IdleState *idleState;
    ProcessingState *processingState;
};

#endif /* CONTEXT_H */

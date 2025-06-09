#ifndef CONTEXT_H
#define CONTEXT_H

#include "IdleState.h"
#include "ProcessingState.h"

template <typename T> class EventQueue;
class Event;
class TimeredEventQueue;
class RadioController;

/**
 * A struct to encapsulate and manage the core dependencies and services
 * of the robot system. This centralizes the "global" objects, making
 * them easier to pass around and manage.
 */
struct Context {
    TimeredEventQueue *timeredEventQueue;
    RadioController *radioController;
    IdleState *idleState;
    ProcessingState *processingState;
};

#endif /* CONTEXT_H */

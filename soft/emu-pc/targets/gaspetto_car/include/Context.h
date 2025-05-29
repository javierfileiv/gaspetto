#ifndef CONTEXT_H
#define CONTEXT_H

#include "IdleState.h"
#include "ProcessingState.h"

class EventQueue;
class MovementController;
class RadioController;
class TimeredEventQueue;

/**
 * @brief A struct to encapsulate and manage the core dependencies and services
 * of the robot system. This centralizes the "global" objects, making
 * them easier to pass around and manage.
 */
struct Context {
    EventQueue *mainEventQueue;
    MovementController *movementController;
    RadioController *radioController;
    TimeredEventQueue *timeredEventQueue;
    IdleState *idleState;
    ProcessingState *processingState;
    uint32_t pwm_freq;
};

#endif /* CONTEXT_H */

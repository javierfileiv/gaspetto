#pragma once

#include "CarEvents.h"
#include "CarStates.h"
#include "EventQueue.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "TimeredEventQueue.h"

class MovementControllerInterface;
class RadioController;

/** @brief Application-specific event queue type. */
using EventQueue = GenericEventQueue<Event>;

/** @brief Application-specific timed event queue type. */
using TimeredEventQueue = GenericTimeredEventQueue<Event>;

/**
 * @brief A struct to encapsulate and manage the core dependencies and services
 * of the robot system. This centralizes the "global" objects, making
 * them easier to pass around and manage.
 */
struct Context {
    EventQueue *mainEventQueue;
    MovementControllerInterface *movementController;
    RadioController *radioController;
    TimeredEventQueue *timeredEventQueue;
    IdleState *idleState;
    ProcessingState *processingState;
    uint32_t pwm_freq;
};

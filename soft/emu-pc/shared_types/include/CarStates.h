#ifndef CAR_STATES_H
#define CAR_STATES_H

#ifndef ARDUINO
#include <stdint.h>
#else
#include "Arduino.h"
#endif

/**
 * @brief State identifiers for GCar FSM.
 * @note Add new states before MAX_STATE_ID.
 */
enum class StateId : uint8_t {
    IDLE, /**< Idle state - low power, waiting for events. */
    PROCESSING, /**< Processing state - handling motor commands. */
    MAX_STATE_ID /**< Sentinel - must be last. */
};

#endif /* CAR_STATES_H */

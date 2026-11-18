#pragma once

#include "CommandId.h"
#include "GenericEvent.h"

#ifndef ARDUINO
#include <stdint.h>
#else
#include "Arduino.h"
#endif

/**
 * @brief Event type identifiers.
 */
enum class EventId : uint8_t {
    NONE, /**< No event. */
    TIMER_ELAPSED, /**< Timer expiration. */
    ACTION, /**< Motor command. */
    BUTTON_PRESSED, /**< Button press. */
    RADIO_TX, /**< Radio transmission. */
    DFU_REQUEST, /**< Enter the STM32 DFU bootloader. */
    IRQ_TEST, /**< Report what the radio IRQ pin saw. */
};

/**
 * @brief Command identifiers for ACTION events.
 */
/**
 * @brief Application-specific Event type.
 */
using Event = GenericEvent<EventId, CommandId>;

/**
 * @brief Convert EventId to string for debugging.
 */
inline const char *eventIdToString(EventId id)
{
    switch (id) {
    case EventId::NONE:
        return "NONE";
    case EventId::TIMER_ELAPSED:
        return "TIMER_ELAPSED";
    case EventId::ACTION:
        return "ACTION";
    case EventId::BUTTON_PRESSED:
        return "BUTTON_PRESSED";
    case EventId::RADIO_TX:
        return "RADIO_TX";
    case EventId::DFU_REQUEST:
        return "DFU_REQUEST";
    case EventId::IRQ_TEST:
        return "IRQ_TEST";
    default:
        return "UNKNOWN";
    }
}

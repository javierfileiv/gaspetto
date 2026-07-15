#ifndef EVENT_H
#define EVENT_H

#ifndef ARDUINO
#include <stdint.h>
#else
#include "Arduino.h"
#endif

/**
 * @brief Packed event structure for radio transmission (2 bytes).
 *
 * Generic packet format - interpretation of fields depends on application.
 */
struct __attribute__((packed)) EventPacket {
    uint8_t eventId;
    uint8_t payload;
};

/**
 * @brief Generic event class for state machine communication.
 *
 * Lightweight event object (no vtable) suitable for embedded use.
 * Template parameters allow application-specific event and payload types.
 * Can be serialized for radio transmission via toPacket()/fromPacket().
 *
 * @tparam EventIdT   Enum type for event identifiers (e.g., EventId)
 * @tparam PayloadT   Enum type for event payload (e.g., CommandId)
 */
template <typename EventIdT, typename PayloadT> class GenericEvent {
public:
    GenericEvent(EventIdT id = EventIdT{}, PayloadT payload = PayloadT{})
            : eventId_(id)
            , payload_(payload)
    {
    }

    ~GenericEvent() = default;

    EventIdT getEventId() const
    {
        return eventId_;
    }
    PayloadT getPayload() const
    {
        return payload_;
    }

    void setEventId(EventIdT id)
    {
        eventId_ = id;
    }
    void setPayload(PayloadT payload)
    {
        payload_ = payload;
    }

    /**
     * @brief Serialize to packet for transmission.
     */
    void toPacket(EventPacket &packet) const
    {
        packet.eventId = static_cast<uint8_t>(eventId_);
        packet.payload = static_cast<uint8_t>(payload_);
    }

    /**
     * @brief Deserialize from received packet.
     */
    static GenericEvent fromPacket(const EventPacket &packet)
    {
        return GenericEvent(static_cast<EventIdT>(packet.eventId),
                            static_cast<PayloadT>(packet.payload));
    }

    /** @brief Get serialized packet size. */
    static constexpr uint8_t packetSize()
    {
        return sizeof(EventPacket);
    }

private:
    EventIdT eventId_;
    PayloadT payload_;
};

#endif /* EVENT_H */

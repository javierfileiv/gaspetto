#pragma once

#include "Arduino.h"
#include "CarEvents.h"
#include "CommandPacket.h"
#include "EventQueue.h"
#include "Log.h"
#include "RadioProtocol.h"

#include <stdint.h>

/** @brief Application-specific event queue type for RadioController. */
using EventQueue = GenericEventQueue<Event>;

/* Enum for radio-specific events. */
enum class RadioEventId : uint8_t { RADIO_RX, RADIO_TX };

class RF24;

class RadioController : public Log {
public:
    RadioController(RF24 &radio, EventQueue *gaspettoQueue, const uint8_t writing_addr[5],
                    const uint8_t reading_addr[5]);
    void init();
    void processRadio();
    void sendEvent(Event evt);
    EventQueue *getRadioQueue();
    bool sendBuffer(const void *data, uint8_t len);

    /**
     * sendTelemetry(): Send telemetry packet via radio
     * @param telemetry: The telemetry data to send
     * @return true if successfully sent, false otherwise
     */
    bool sendTelemetry(const TelemetryPacket &telemetry);

private:
    /**
     * @brief Decode a CommandPacket and enqueue each command as EVENT_ACTION.
     * @param packet The CommandPacket to decode.
     */
    void decodeCommandPacket(const CommandPacket &packet);

    RF24 &_radio;
    uint8_t writing_addr[5];
    uint8_t reading_addr[5];
    EventQueue radioQueue;
    EventQueue *gaspettoQueue;
};

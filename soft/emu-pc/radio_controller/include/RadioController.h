#pragma once

#include "Arduino.h"
#include "CommandPacket.h"
#include "EventQueue.h"
#include "Events.h"
#include "Log.h"
#include "OtaProtocol.h"
#include "RadioProtocol.h"

#include <stdint.h>

/** @brief Application-specific event queue type for RadioController. */
using EventQueue = GenericEventQueue<Event>;

/* Enum for radio-specific events. */
enum class RadioEventId : uint8_t { RADIO_RX, RADIO_TX };

class RF24;

class RadioController : public Log {
public:
    using OtaHandler = void (*)(const OtaPacket &packet);
    using ProgramDoneHandler = void (*)(void);

    RadioController(RF24 &radio, EventQueue *gaspettoQueue, const uint8_t writing_addr[5],
                    const uint8_t reading_addr[5]);
    void init();
    void processRadio();
    void sendEvent(Event evt);
    EventQueue *getRadioQueue();
    bool sendBuffer(const void *data, uint8_t len);
    bool isChipConnected();

    /** setOtaHandler(): Install the receiver of an image transfer
     * @handler: called for every OTA packet; null drops them
     */
    void setOtaHandler(OtaHandler handler);

    /** setProgramDoneHandler(): Install the receiver of the car's end of
     * program notice; null ignores it */
    void setProgramDoneHandler(ProgramDoneHandler handler);

    /** setLogPeerPrefix(): Name the device whose log lines arrive on the
     * radio, so they can be told apart from this device's own lines */
    void setLogPeerPrefix(const char *prefix);

    /** lastVersionMs(): when the last Version answer arrived, 0 if none yet;
     * a box uses its age as the witness that the other side is alive */
    uint32_t lastVersionMs() const
    {
        return versionLastMs;
    }

    /** requestVersionLog(): print the next Version answer even when it is the
     * build the periodic ping already showed */
    void requestVersionLog()
    {
        versionLogForced = true;
    }

    /** setQuiet(): Leave RX mode and refuse every send, so this device does
     * not disturb another link on the bench; false listens again */
    void setQuiet(bool quiet);

    bool isQuiet() const
    {
        return radioQuiet;
    }

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

    /* Reassemble one received log fragment and print the line once the
     * final piece arrives */
    void handleLogFragment(const uint8_t *buffer);

    /* Answer a VersionRequest with this build's identity */
    void respondVersion();

    RF24 &_radio;
    uint8_t writing_addr[5];
    uint8_t reading_addr[5];
    EventQueue radioQueue;
    EventQueue *gaspettoQueue;
    OtaHandler otaHandler = nullptr;
    ProgramDoneHandler programDoneHandler = nullptr;
    const char *logPeerPrefix = nullptr;
    bool otaWarningShown = false;
    bool radioQuiet = false;
    uint8_t logLine[64];
    uint8_t logLineLen;
    uint32_t versionLastMs = 0;
    uint16_t versionBuild = 0;
    uint8_t versionWire = 0;
    char versionHash[28] = "";
    bool versionLogForced = false;
};

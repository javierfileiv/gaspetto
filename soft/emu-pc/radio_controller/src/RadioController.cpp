#include "RadioController.h"

#include "Arduino.h"
#include "Events.h"
#include "RadioProtocol.h"
#include "config_radio.h"

#include <cstring>

const uint64_t address = 0xdeadbeef11LL;
float payload = 0.0;

RadioController::RadioController(RF24 &radio, EventQueue *gaspettoQueue,
                                 const uint8_t writing_addr[5], const uint8_t reading_addr[5])
        : gaspettoQueue(gaspettoQueue)
        , _radio(radio)
{
    for (int i = 0; i < 5; i++) {
        this->writing_addr[i] = writing_addr[i];
        this->reading_addr[i] = reading_addr[i];
    }
    logLineLen = 0;
    std::memset(logLine, 0, sizeof(logLine));
}

void RadioController::init()
{
    LOG(F("Writing address: "));
    for (int i = 0; i < 5; i++) {
        LOG(writing_addr[i], HEX);
        LOG(F(" "));
    }
    LOGLN();
    LOG(F("Reading address: "));
    for (int i = 0; i < 5; i++) {
        LOG(reading_addr[i], HEX);
        LOG(F(" "));
    }
    LOGLN();
    if (!_radio.begin()) {
        LOGLN(F("radio hardware is not responding!!"));
        return;
    }
    _radio.setPALevel(PA_LEVEL);
    _radio.setDataRate(DATA_RATE);
    _radio.setAddressWidth(5);
    /* Support both EventPacket (2 bytes) and CommandPacket (32 bytes). */
    _radio.setPayloadSize(sizeof(CommandPacket));
    _radio.openWritingPipe(writing_addr);
    _radio.openReadingPipe(1, reading_addr);
    _radio.printDetails();
    _radio.printPrettyDetails();
#if NFR_IRQ
    /* Set up interrupt for RX. */
    pinMode(NRF_IRQ, INPUT);
    attachInterrupt(digitalPinToInterrupt(NRF_IRQ), ISR, FALLING);
    radio.maskIRQ(0, 1, 1);
#endif
    _radio.powerUp();
    _radio.startListening();
}

void RadioController::processRadio()
{
    uint8_t buffer[sizeof(CommandPacket)];
    uint8_t pipe;

    /* RX processing. */
    if (_radio.available(&pipe)) {
        LOG(F("Radio RX: pipe "));
        LOG(static_cast<int>(pipe));
        LOGLN();
        _radio.read(buffer, sizeof(buffer)); /* Fetch payload from FIFO. */

        /* Byte 0 names the packet: a program keeps its command count
         * there (never above 31) and every other kind carries a
         * PacketTag, so one switch routes them all. The old heuristic
         * read byte 1, which is a CommandId on a program and on an
         * event alike: an event carrying a low CommandId was decoded as
         * a program. */
        switch (static_cast<PacketTag>(buffer[0])) {
        case PacketTag::Event: {
            EventPacket eventPacket;
            std::memcpy(&eventPacket, buffer, sizeof(eventPacket));
            LOG(F("Radio RX: EventPacket "));
            LOG(eventIdToString(static_cast<EventId>(eventPacket.eventId)));
            LOG(F(" "));
            LOG(commandIdToString(static_cast<CommandId>(eventPacket.payload)));
            LOGLN(F("."));
            if (gaspettoQueue == nullptr) {
                LOGLN(F("Radio RX: app queue unavailable."));
                break;
            }
            if (gaspettoQueue->IsFull()) {
                LOGLN(F("Radio RX: app queue full."));
                break;
            }
            /* Post to active object queue. */
            Event evt = Event::fromPacket(eventPacket);
            gaspettoQueue->enqueue(evt);
            LOG(F("Radio RX: queued event; app queue size="));
            LOGLN(static_cast<int>(gaspettoQueue->GetSize()));
            break;
        }
        case PacketTag::Telemetry: {
            TelemetryPacket telemetry;
            std::memcpy(&telemetry, buffer, sizeof(telemetry));
            /* Watched, not acted on: the command that hands telemetry to
             * the application is still to be written */
            LOG(F("Radio RX: Telemetry yaw="));
            LOG(telemetry.yaw, 1);
            LOG(F(" tgt="));
            LOG(telemetry.targetYaw, 1);
            LOG(F(" state="));
            LOGLN(static_cast<int>(telemetry.state));
            break;
        }
        case PacketTag::Log:
            handleLogFragment(buffer);
            break;
        default:
            /* Not a tag: byte 0 is the command count of a program, and a
             * count above the maximum means an unknown tag after all */
            if (buffer[0] > BOX_MAX_PAYLOAD_COMMANDS) {
                LOGLN(F("Radio RX: unknown packet tag, dropped."));
                break;
            }
            CommandPacket *boxPacket = reinterpret_cast<CommandPacket *>(buffer);
            LOG(F("Radio RX: CommandPacket count="));
            LOGLN(static_cast<int>(boxPacket->count));
            decodeCommandPacket(*boxPacket);
            break;
        }
    }
    /* TX processing. Only send the first one. */
    if (!radioQueue.IsEmpty()) {
        EventPacket packet;
        Event evt;

        radioQueue.dequeue(evt);
        LOG(F("RadioController::ProcessRadio: "));
        LOG(eventIdToString(evt.getEventId()));
        LOG(F(" - "));
        LOGLN(commandIdToString(evt.getPayload()));
        evt.toPacket(packet);
        packet.tag = static_cast<uint8_t>(PacketTag::Event);
        /* Stop listening. */
        _radio.stopListening();
        _radio.write(&packet, sizeof(packet));
        LOG(F("RadioController::ProcessRadio: Sent EventId:"));
        LOG(eventIdToString(static_cast<EventId>(packet.eventId)));
        LOG(F(" CommandId:"));
        LOG(commandIdToString(static_cast<CommandId>(packet.payload)));
        LOGLN(F("."));
        _radio.startListening();
    }
}

void RadioController::sendEvent(Event evt)
{
    if (radioQueue.IsFull()) {
        LOGLN(F("RadioController::SendEvent: Queue is full."));
        return;
    }
    radioQueue.enqueue(evt);
}

EventQueue *RadioController::getRadioQueue()
{
    return &radioQueue;
}

bool RadioController::sendBuffer(const void *data, uint8_t len)
{
    _radio.stopListening();
    const bool result = _radio.write(data, len);
    _radio.startListening();

    if (!result) {
        LOGLN(F("RadioController::sendBuffer failed"));
    }

    return result;
}

bool RadioController::isChipConnected()
{
    return _radio.isChipConnected();
}

bool RadioController::sendTelemetry(const TelemetryPacket &telemetry)
{
    /* One address for everything this device sends: sendBuffer() switches
     * no pipe, so the telemetry rides writing_addr (_box_ for the car) like
     * an event does, and the auto-ack comes back on pipe 0 as always. The
     * tag is stamped here so no caller has to know the wire format. */
    TelemetryPacket packet = telemetry;
    packet.tag = static_cast<uint8_t>(PacketTag::Telemetry);

    const bool result = sendBuffer(&packet, sizeof(packet));
    if (!result) {
        LOGLN(F("Telemetry send failed"));
    }
    return result;
}

void RadioController::decodeCommandPacket(const CommandPacket &packet)
{
    LOG(F("Radio RX: decode command count="));
    LOG(static_cast<int>(packet.count));
    LOGLN();

    if (packet.count == 0 || packet.count > BOX_MAX_PAYLOAD_COMMANDS) {
        if (packet.count != 0) {
            LOGLN(F("Invalid count in CommandPacket"));
        }
        return;
    }

    /* Enqueue each command as an ACTION event. */
    for (uint8_t i = 0; i < packet.count; ++i) {
        if (gaspettoQueue == nullptr) {
            LOGLN(F("Radio RX: app queue unavailable."));
            break;
        }
        if (gaspettoQueue->IsFull()) {
            LOGLN(F("Radio RX: app queue full."));
            break;
        }

        CommandId cmdId = static_cast<CommandId>(packet.commands[i]);
        LOG(F("Radio RX: command["));
        LOG(static_cast<int>(i));
        LOG(F("] "));
        LOG(commandIdToString(cmdId));

        if (cmdId == CommandId::NONE)
            continue;
        Event evt(EventId::ACTION, cmdId);
        gaspettoQueue->enqueue(evt);
        LOG(F(" queued; app queue size="));
        LOGLN(static_cast<int>(gaspettoQueue->GetSize()));
    }
}

void RadioController::handleLogFragment(const uint8_t *buffer)
{
    const uint8_t flags = buffer[1];
    const uint8_t dataLen = flags & 0x7FU;
    const bool hasMore = (flags & 0x80U) != 0U;
    const uint8_t room = static_cast<uint8_t>(sizeof(logLine) - logLineLen);
    const uint8_t copy = (dataLen < room) ? dataLen : room;

    if (copy < dataLen) {
        LOGLN(F("Radio RX: log line too long, truncated."));
    }

    std::memcpy(&logLine[logLineLen], &buffer[2], copy);
    logLineLen = static_cast<uint8_t>(logLineLen + copy);

    if (hasMore) {
        return;
    }

    for (uint8_t i = 0; i < logLineLen; ++i) {
        LOG(static_cast<char>(logLine[i]));
    }
    LOGLN();
    logLineLen = 0;
}

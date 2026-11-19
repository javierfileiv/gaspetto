#include "RadioController.h"

#include "Arduino.h"
#include "Events.h"
#include "FirmwareVersion.h"
#include "RadioProtocol.h"
#include "config_radio.h"
#include "radio_send.h"

#include <cstring>

const uint64_t address = 0xdeadbeef11LL;
float payload = 0.0;

#if defined(NRF_IRQ_PIN)
/* Written by the pin interrupt and read by processRadio(): the count is one
 * per IRQ assertion, not one per packet, because the line stays low until
 * RX_DR is cleared, so a burst already sitting in the FIFO produces a single
 * falling edge */
volatile uint32_t irq_edge_count = 0;
volatile uint32_t irq_last_edge_ms = 0;

/* Free function, not a method: attachInterrupt() takes a plain function
 * pointer. It only records, because starting an SPI transfer inside an
 * interrupt would stall the bus the main loop is also using */
void radio_controller_count_irq_edge()
{
    ++irq_edge_count;
    irq_last_edge_ms = millis();
}
#endif

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
#ifdef ARDUINO
    /* A module can come back from a test with the continuous-carrier bits set,
     * and it keeps RF_SETUP across MCU resets because its supply never drops.
     * In that mode the chip answers SPI and transmits a carrier instead of
     * packets, so the link is silent with no other symptom: leave the mode
     * explicitly before trusting the radio */
    _radio.stopConstCarrier();
#endif
    _radio.setPALevel(PA_LEVEL);
    _radio.setDataRate(DATA_RATE);
    _radio.setAddressWidth(5);
    /* Three retransmissions instead of the library's fifteen: a packet the
     * other side never acknowledges must not cost 22 ms, and the bounded send
     * in sendBuffer needs MAX_RT to arrive well inside its timeout */
    _radio.setRetries(2, 3);
    /* Support both EventPacket (2 bytes) and CommandPacket (32 bytes). */
    _radio.setPayloadSize(sizeof(CommandPacket));
    _radio.openWritingPipe(writing_addr);
    _radio.openReadingPipe(1, reading_addr);
    _radio.printDetails();
    _radio.printPrettyDetails();
#if defined(NRF_IRQ_PIN)
    /* Attach before startListening(): a packet landing in between would have
     * no interrupt to announce it. The pull-up is set first because
     * attachInterrupt() keeps the pull it finds, and it holds an unwired pin
     * high instead of letting it float into phantom edges */
    irq_edge_count = 0;
    irq_last_edge_ms = 0;
    pinMode(NRF_IRQ_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), radio_controller_count_irq_edge, FALLING);
    _radio.clearStatusFlags();
    /* Only RX_DR reaches the pin: with TX_DS unmasked, every log line this
     * device sends would count as a received packet */
    _radio.setStatusFlags(RF24_RX_DR);
#endif
    _radio.powerUp();
    _radio.startListening();
#ifdef GASPETTO_LOG_OVER_NRF24
    /* The chip answered begin(), so fragments can be framed and sent from
     * here on: until this point the log output mirrored onto the console */
    Log::markNrf24Ready();
#endif
}

void RadioController::processRadio()
{
    uint8_t buffer[sizeof(CommandPacket)];
    uint8_t pipe;

    /* RX processing. */
    if (_radio.available(&pipe)) {
        _radio.read(buffer, sizeof(buffer)); /* Fetch payload from FIFO. */

        /* An image transfer is thousands of packets: a log line per packet
         * would make the receiver deaf while it transmits the line, on the
         * very channel the transfer is using */
        if (buffer[0] != static_cast<uint8_t>(PacketTag::Ota)) {
            LOG(F("Radio RX: pipe "));
            LOG(static_cast<int>(pipe));
            LOGLN();
        }

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

            /* A diagnostic, not an application event: the car reports what its
             * IRQ pin saw and the state machine never hears about it. The
             * packet causing this report landed a moment ago, so a wired pin
             * has already counted at least one falling edge */
            if (static_cast<EventId>(eventPacket.eventId) == EventId::IRQ_TEST) {
#if defined(NRF_IRQ_PIN)
                LOG(F("[irq] A4 falling edges="));
                LOG(static_cast<int>(irq_edge_count));
                LOG(F(", last at "));
                LOG(static_cast<int>(irq_last_edge_ms));
                LOGLN(F(" ms"));
#else
                LOGLN(F("[irq] no IRQ pin on this target"));
#endif
                break;
            }

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
        case PacketTag::VersionRequest:
            respondVersion();
            break;
        case PacketTag::Version: {
            VersionPacket version;
            std::memcpy(&version, buffer, sizeof(version));
            LOG(F("Radio RX: Version build="));
            LOGLN(static_cast<int>(version.build));
            break;
        }
        case PacketTag::ProgramDone:
            /* The car finished the batch it was given: let the application
             * animate it, it knows what a finished program means */
            LOGLN(F("Radio RX: program done"));
            if (programDoneHandler != nullptr) {
                programDoneHandler();
            }
            break;
        case PacketTag::Ota: {
            OtaPacket packet;
            std::memcpy(&packet, buffer, sizeof(packet));
            if (otaHandler != nullptr) {
                otaHandler(packet);
            } else if (!otaWarningShown) {
                /* One line is enough: a full transfer is thousands of packets
                 * and a log per packet would flood the radio it is using */
                otaWarningShown = true;
                LOGLN(F("Radio RX: OTA packet with no receiver, dropped."));
            }
            break;
        }
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
        /* Same bounded path as every other send: the caller reports whether
         * the other side acknowledged, and a dead chip cannot hold the loop */
        sendBuffer(&packet, sizeof(packet));
        LOG(F("RadioController::ProcessRadio: Sent EventId:"));
        LOG(eventIdToString(static_cast<EventId>(packet.eventId)));
        LOG(F(" CommandId:"));
        LOG(commandIdToString(static_cast<CommandId>(packet.payload)));
        LOGLN(F("."));
    }
}

void RadioController::respondVersion()
{
    VersionPacket packet = {};

    packet.tag = static_cast<uint8_t>(PacketTag::Version);
    packet.wire = kWireProtocolVersion;
    packet.build = static_cast<uint16_t>(FW_VERSION);
    /* The revision is a short hash, "-dirty" included when the tree was not
     * clean at build time. The field starts zeroed, so an over-long string is
     * truncated instead of left unterminated */
    std::strncpy(packet.version, FW_BUILD, sizeof(packet.version) - 1);

    sendBuffer(&packet, sizeof(packet));
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
    /* RF24's write() returns as soon as the payload is loaded, so it reports
     * success before the acknowledgement can be known. This bounded pair
     * waits for TX_DS or MAX_RT, which makes the verdict real, and gives up
     * after the timeout so a chip that never answers cannot hold the loop.
     *
     * startWrite()'s return value is ignored on purpose: it reports bit 0 of
     * the STATUS register, which is RX_DR, not the TX FIFO's full flag, so a
     * telemetry packet waiting in the RX FIFO made it look like a failed load
     * and the send was reported refused while the car had answered */
    constexpr uint32_t TX_TIMEOUT_MS = 10;

    const bool result = radio_send(_radio, data, len, TX_TIMEOUT_MS);

    if (!result) {
        /* An image transfer fails and retries by design, and a log line would
         * take two more packets on the channel that is already the bottleneck,
         * which is what turned one lost answer into a storm of losses */
        /* Telemetry is periodic best effort, and a log line per lost sample
         * would add traffic to the channel that just lost one */
        const uint8_t tag = static_cast<const uint8_t *>(data)[0];
        if (tag != static_cast<uint8_t>(PacketTag::Ota) &&
            tag != static_cast<uint8_t>(PacketTag::Telemetry)) {
            LOGLN(F("RadioController::sendBuffer failed"));
        }
    }

    return result;
}

bool RadioController::isChipConnected()
{
    return _radio.isChipConnected();
}

void RadioController::setOtaHandler(OtaHandler handler)
{
    otaHandler = handler;
    otaWarningShown = false;
}

void RadioController::setProgramDoneHandler(ProgramDoneHandler handler)
{
    programDoneHandler = handler;
}

bool RadioController::sendTelemetry(const TelemetryPacket &telemetry)
{
    /* One address for everything this device sends: sendBuffer() switches
     * no pipe, so the telemetry rides writing_addr (_box_ for the car) like
     * an event does, and the auto-ack comes back on pipe 0 as always. The
     * tag is stamped here so no caller has to know the wire format. */
    TelemetryPacket packet = telemetry;
    packet.tag = static_cast<uint8_t>(PacketTag::Telemetry);

    return sendBuffer(&packet, sizeof(packet));
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

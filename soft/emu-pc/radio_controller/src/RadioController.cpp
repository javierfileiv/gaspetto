#include "RadioController.h"

#include "Arduino.h"
#include "CarEvents.h"
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

        /* Heuristic: Detect packet type based on buffer content.
         * EventPacket: eventId (byte 0) = 0-4, payload (byte 1) = CommandId
         * CommandPacket: count (byte 0) = 0-31, commands[0] = CommandId
         * If byte[0] <= 31 AND byte[1] is valid CommandId, assume BoxProgram.
         */
        uint8_t second_byte = buffer[1];
        uint8_t maxCommandId = static_cast<uint8_t>(CommandId::MAX_COMMAND_ID);

        /* Check if it looks like a CommandPacket. */
        if (second_byte < maxCommandId) {
            CommandPacket *boxPacket = reinterpret_cast<CommandPacket *>(buffer);
            LOG(F("Radio RX: CommandPacket count="));
            LOGLN(static_cast<int>(boxPacket->count));
            decodeCommandPacket(*boxPacket);
        } else {
            /* Treat as EventPacket. */
            EventPacket *eventPacket = reinterpret_cast<EventPacket *>(buffer);
            LOG(F("Radio RX: EventPacket "));
            LOG(eventIdToString(static_cast<EventId>(eventPacket->eventId)));
            LOG(F(" "));
            LOG(commandIdToString(static_cast<CommandId>(eventPacket->payload)));
            LOGLN(F("."));
            Event evt = Event::fromPacket(*eventPacket);
            if (gaspettoQueue == nullptr) {
                LOGLN(F("Radio RX: app queue unavailable."));
                return;
            }
            if (gaspettoQueue->IsFull()) {
                LOGLN(F("Radio RX: app queue full."));
                return;
            }
            /* Post to active object queue. */
            gaspettoQueue->enqueue(evt);
            LOG(F("Radio RX: queued event; app queue size="));
            LOGLN(static_cast<int>(gaspettoQueue->GetSize()));
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

bool RadioController::sendTelemetry(const TelemetryPacket &telemetry)
{
    bool result = false;
    /* Switch to telemetry address for transmission. */
    _radio.openWritingPipe(RADIO_ADDR_TLM);
    result = sendBuffer(&telemetry, sizeof(telemetry));
    /* Restore command pipe and resume listening. */
    _radio.openWritingPipe(writing_addr);
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

        Event evt(EventId::ACTION, cmdId);
        gaspettoQueue->enqueue(evt);
        LOG(F(" queued; app queue size="));
        LOGLN(static_cast<int>(gaspettoQueue->GetSize()));
    }
}

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
    log(F("Writing address: "));
    for (int i = 0; i < 5; i++) {
        log(writing_addr[i], HEX);
        log(F(" "));
    }
    logln();
    log(F("Reading address: "));
    for (int i = 0; i < 5; i++) {
        log(reading_addr[i], HEX);
        log(F(" "));
    }
    logln();
    if (!_radio.begin()) {
        logln(F("radio hardware is not responding!!"));
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
        logln(F("ProcessRadio():"));
        _radio.read(buffer, sizeof(buffer)); /* Fetch payload from FIFO. */

        /* Heuristic: Detect packet type based on buffer content.
         * EventPacket: eventId (byte 0) = 0-4, payload (byte 1) = CommandId
         * CommandPacket: count (byte 0) = 0-31, commands[0] = CommandId
         * If byte[0] <= 31 AND byte[1] is valid CommandId, assume BoxProgram.
         */
        uint8_t first_byte = buffer[0];
        uint8_t second_byte = buffer[1];
        uint8_t maxCommandId = static_cast<uint8_t>(CommandId::MAX_COMMAND_ID);

        /* Check if it looks like a CommandPacket. */
        if (second_byte < maxCommandId) {
            logln(F("Detected CommandPacket"));
            CommandPacket *boxPacket = reinterpret_cast<CommandPacket *>(buffer);
            decodeCommandPacket(*boxPacket);
        } else {
            /* Treat as EventPacket. */
            logln(F("Detected EventPacket"));
            EventPacket *eventPacket = reinterpret_cast<EventPacket *>(buffer);
            log(F("Received EventId:"));
            log(eventIdToString(static_cast<EventId>(eventPacket->eventId)));
            log(F(" CommandId:"));
            log(commandIdToString(static_cast<CommandId>(eventPacket->payload)));
            logln(F("."));
            Event evt = Event::fromPacket(*eventPacket);
            if (radioQueue.IsFull()) {
                logln(F("RadioController::ProcessRadio: Queue is full."));
                return;
            }
            /* Post to active object queue. */
            gaspettoQueue->enqueue(evt);
        }
    }
    /* TX processing. Only send the first one. */
    if (!radioQueue.IsEmpty()) {
        EventPacket packet;
        Event evt;

        radioQueue.dequeue(evt);
        log(F("RadioController::ProcessRadio: "));
        log(eventIdToString(evt.getEventId()));
        log(F(" - "));
        logln(commandIdToString(evt.getPayload()));
        evt.toPacket(packet);
        /* Stop listening. */
        _radio.stopListening();
        _radio.write(&packet, sizeof(packet));
        log(F("RadioController::ProcessRadio: Sent EventId:"));
        log(eventIdToString(static_cast<EventId>(packet.eventId)));
        log(F(" CommandId:"));
        log(commandIdToString(static_cast<CommandId>(packet.payload)));
        logln(F("."));
        _radio.startListening();
    }
}

void RadioController::sendEvent(Event evt)
{
    if (radioQueue.IsFull()) {
        logln(F("RadioController::SendEvent: Queue is full."));
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
        logln(F("RadioController::sendBuffer failed"));
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
        logln(F("Telemetry send failed"));
    }
    return result;
}

void RadioController::decodeCommandPacket(const CommandPacket &packet)
{
    log(F("CommandPacket: count="));
    log(packet.count);
    logln();

    if (packet.count == 0 || packet.count > BOX_MAX_PAYLOAD_COMMANDS) {
        if (packet.count != 0) {
            logln(F("Invalid count in CommandPacket"));
        }
        return;
    }

    /* Enqueue each command as an ACTION event. */
    for (uint8_t i = 0; i < packet.count; ++i) {
        if (gaspettoQueue->IsFull()) {
            logln(F("RadioController::decodeCommandPacket: Queue is full."));
            break;
        }

        CommandId cmdId = static_cast<CommandId>(packet.commands[i]);
        log(F("  Command["));
        log(i);
        log(F("]: "));
        log(commandIdToString(cmdId));
        logln();

        Event evt(EventId::ACTION, cmdId);
        gaspettoQueue->enqueue(evt);
    }
}

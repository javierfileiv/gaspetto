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
        while (1) {
        }
    }
    _radio.setPALevel(PA_LEVEL);
    _radio.setDataRate(DATA_RATE);
    _radio.setAddressWidth(5);
    _radio.setPayloadSize(Event::packetSize());
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
    EventPacket packet;
    uint8_t pipe;

    /* RX processing. */
    if (_radio.available(&pipe)) {
        logln(F("ProcessRadio():"));
        _radio.read(&packet, sizeof(packet)); /* Fetch payload from FIFO. */
        log(F("Received EventId:"));
        log(eventIdToString(static_cast<EventId>(packet.eventId)));
        log(F(" CommandId:"));
        log(commandIdToString(static_cast<CommandId>(packet.payload)));
        logln(F("."));
        Event evt = Event::fromPacket(packet);
        if (radioQueue.IsFull()) {
            logln(F("RadioController::ProcessRadio: Queue is full."));
            return;
        }
        /* Post to active object queue. */
        gaspettoQueue->enqueue(evt);
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

bool RadioController::sendTelemetry(const TelemetryPacket &telemetry)
{
    /* Stop listening to transmit. */
    _radio.stopListening();

    /* Switch to telemetry address for transmission. */
    _radio.openWritingPipe(RADIO_ADDR_TLM);

    bool result = _radio.write(&telemetry, sizeof(telemetry));

    /* Restore command pipe and resume listening. */
    _radio.openWritingPipe(writing_addr);
    _radio.startListening();

    if (!result) {
        logln(F("Telemetry send failed"));
    }

    return result;
}

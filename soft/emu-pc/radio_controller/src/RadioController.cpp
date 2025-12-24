#include "RadioController.h"

#include "Arduino.h"
#include "Event.h"
#include "config_radio.h"

const uint64_t address = 0xdeadbeef11LL;
float payload = 0.0;

RadioController::RadioController(RF24 &radio, EventQueue *queue, const uint8_t writingAddr[5],
                                 const uint8_t readingAddr[5])
        : _radio(radio)
        , gaspettoQueue(queue)
{
    for (int i = 0; i < 5; i++) {
        this->writing_addr[i] = writingAddr[i];
        this->reading_addr[i] = readingAddr[i];
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
        log(Event::eventIdToString(static_cast<EventId>(packet.eventId)));
        log(F(" CommandId:"));
        log(Event::commandIdToString(static_cast<CommandId>(packet.commandId)));
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
        log(Event::eventIdToString(evt.getEventId()));
        log(F(" - "));
        logln(Event::commandIdToString(evt.getCommand()));
        evt.toPacket(packet);
        /* Stop listening. */
        _radio.stopListening();
        _radio.write(&packet, sizeof(packet));
        log(F("RadioController::ProcessRadio: Sent EventId:"));
        log(Event::eventIdToString(static_cast<EventId>(packet.eventId)));
        log(F(" CommandId:"));
        log(Event::commandIdToString(static_cast<CommandId>(packet.commandId)));
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

#include "RadioController.h"

#include "Arduino.h"
#include "Event.h"
#include "config_radio.h"

#include <printf.h>

RadioController::RadioController(RF24 &radio, const uint8_t writing_addr[5],
                                 const uint8_t reading_addr[5])
        : _radio(radio)
        , radioQueue(RADIO_QUEUE_SIZE)
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
    printf_begin();
    _radio.setPALevel(PA_LEVEL);
    _radio.setDataRate(DATA_RATE);
    _radio.setPayloadSize(Event::packetSize());
    _radio.openWritingPipe(gaspetto_box_pipe_name);
    _radio.openReadingPipe(1, gaspetto_car_pipe_name);
    _radio.setRetries(3, 5);
    _radio.setAutoAck(true);
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
    uint8_t pipe;
    /* RX processing. */
    if (_radio.available(&pipe)) {
        EventPacket packet;

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
    /* TX processing. */
    if (!radioQueue.IsEmpty()) {
        TelemetryData evt;

        radioQueue.dequeue(evt);
        /* Stop listening. */
        _radio.stopListening();

        _radio.startListening();
    }
}

EventQueue<TelemetryData> *RadioController::getRadioQueue()
{
    return &radioQueue;
}

bool RadioController::postRadioEvent(TelemetryData evt)
{
    if (radioQueue.IsFull()) {
        logln(F("RadioController::PostEvent: Gaspetto queue is full."));
        return false;
    }
    return radioQueue.enqueue(evt);
}

void RadioController::setEventQueue(EventQueue<Event> *queue)
{
    gaspettoQueue = queue;
}

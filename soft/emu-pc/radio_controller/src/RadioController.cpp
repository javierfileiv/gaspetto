#include "RadioController.h"

#include "Arduino.h"
#include "Event.h"

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

void RadioController::Init()
{
    Serial.print(F("Writing address: "));
    for (int i = 0; i < 5; i++) {
        Serial.print(writing_addr[i], HEX);
        Serial.print(F(" "));
    }
    Serial.println();
    Serial.print(F("Reading address: "));
    for (int i = 0; i < 5; i++) {
        Serial.print(reading_addr[i], HEX);
        Serial.print(F(" "));
    }
    Serial.println();
    if (!_radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
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

void RadioController::ProcessRadio()
{
    EventPacket packet;
    uint8_t pipe;

    /* RX processing. */
    if (_radio.available(&pipe)) {
        Serial.println(F("ProcessRadio():"));
        _radio.read(&packet, sizeof(packet)); /* Fetch payload from FIFO. */
        Serial.print(F("Received EventId:"));
        Serial.print(EventQueue::eventIdToString(static_cast<EventId>(packet.eventId)));
        Serial.print(F(" CommandId:"));
        Serial.print(EventQueue::commandIdToString(static_cast<CommandId>(packet.commandId)));
        Serial.println(F("."));
        Event evt = Event::fromPacket(packet);
        if (radioQueue.IsFull()) {
            Serial.println(F("RadioController::ProcessRadio: Queue is full."));
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
        Serial.print(F("RadioController::ProcessRadio: "));
        Serial.print(EventQueue::eventIdToString(evt.getEventId()));
        Serial.print(F(" - "));
        Serial.println(EventQueue::commandIdToString(evt.getCommand()));
        evt.toPacket(packet);
        /* Stop listening. */
        _radio.stopListening();
        _radio.write(&packet, sizeof(packet));
        Serial.print(F("RadioController::ProcessRadio: Sent EventId:"));
        Serial.print(EventQueue::eventIdToString(static_cast<EventId>(packet.eventId)));
        Serial.print(F(" CommandId:"));
        Serial.print(EventQueue::commandIdToString(static_cast<CommandId>(packet.commandId)));
        Serial.println(F("."));
        _radio.startListening();
    }
}

void RadioController::SendEvent(Event evt)
{
    if (radioQueue.IsFull()) {
        Serial.println(F("RadioController::SendEvent: Queue is full."));
        return;
    }
    radioQueue.enqueue(evt);
}

EventQueue *RadioController::getRadioQueue()
{
    return &radioQueue;
}

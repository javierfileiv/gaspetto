#ifndef RADIO_CONTROLLER_H
#define RADIO_CONTROLLER_H

#include "Arduino.h"
#include "EventQueue.h"
#include "RF24.h"

#include <stdint.h>

const uint32_t CE_PIN = PB15;
const uint32_t CSN_PIN = PA4;
const uint32_t PA_LEVEL = RF24_PA_LOW;
const rf24_datarate_e DATA_RATE = RF24_1MBPS;

/* Enum for radio-specific events. */
enum class RadioEventId : uint8_t { RADIO_RX, RADIO_TX };

class RadioController {
public:
    RadioController(RF24 &radio, EventQueue *gaspettoQueue, const uint8_t writing_addr[5],
                    const uint8_t reading_addr[5]);
    void Init();
    void ProcessRadio();
    void SendEvent(Event evt);
    EventQueue *getRadioQueue();

private:
    RF24 &_radio;
    uint8_t writing_addr[5];
    uint8_t reading_addr[5];
    EventQueue radioQueue;
    EventQueue *gaspettoQueue;
};

#endif /* RADIO_CONTROLLER_H. */

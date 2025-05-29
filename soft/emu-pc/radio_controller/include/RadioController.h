#ifndef RADIO_CONTROLLER_H
#define RADIO_CONTROLLER_H

#include "Arduino.h"
#include "EventQueue.h"
#include "Log.h"

#include <stdint.h>

/* Enum for radio-specific events. */
enum class RadioEventId : uint8_t { RADIO_RX, RADIO_TX };

class RF24;

class RadioController : public Log {
public:
    RadioController(RF24 &radio, EventQueue *gaspettoQueue, const uint8_t writing_addr[5],
                    const uint8_t reading_addr[5]);
    void init();
    void processRadio();
    void sendEvent(Event evt);
    EventQueue *getRadioQueue();

private:
    RF24 &_radio;
    uint8_t writing_addr[5];
    uint8_t reading_addr[5];
    EventQueue radioQueue;
    EventQueue *gaspettoQueue;
};

#endif /* RADIO_CONTROLLER_H. */

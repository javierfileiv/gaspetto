#ifndef RADIO_CONTROLLER_H
#define RADIO_CONTROLLER_H

#include "EventQueue.h"
#include "RF24.h"

#include <stdint.h>

extern const int CE_PIN;
extern const int CSN_PIN;
extern const int PA_LEVEL;
extern const rf24_datarate_e DATA_RATE;

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

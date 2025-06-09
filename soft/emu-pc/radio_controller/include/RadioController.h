#ifndef RADIO_CONTROLLER_H
#define RADIO_CONTROLLER_H

#include "Arduino.h"
#include "Event.h"
#include "EventQueue.h"
#include "Log.h"
#include "LogData.h"
#include "config_event.h"

#include <stdint.h>

class RF24;

class RadioController : public Log {
public:
    RadioController(RF24 &radio, const uint8_t writing_addr[5], const uint8_t reading_addr[5]);
    void init();
    void processRadio();
    EventQueue<TelemetryData> *getRadioQueue();
    bool postRadioEvent(TelemetryData evt);
    void setEventQueue(EventQueue<Event> *queue);

private:
    RF24 &_radio;
    uint8_t writing_addr[5];
    uint8_t reading_addr[5];
    EventQueue<TelemetryData> radioQueue;
    EventQueue<Event> *gaspettoQueue;
};

#endif /* RADIO_CONTROLLER_H. */

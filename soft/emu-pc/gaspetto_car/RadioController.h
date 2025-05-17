#ifndef RADIO_CONTROLLER_H
#define RADIO_CONTROLLER_H

#include "EventQueue.h"

#include <stdint.h>

// Enum for radio-specific events
enum class RadioEventId : uint8_t { RADIO_RX, RADIO_TX };

class RadioController {
public:
    RadioController(EventQueue *gaspettoQueue);
    void process_radio();
    void send_action(uint8_t action);
    EventQueue *getRadioQueue();

private:
    EventQueue radioQueue;
    EventQueue *gaspettoCarQueue;
};

#endif // RADIO_CONTROLLER_H

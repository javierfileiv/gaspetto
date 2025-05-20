#ifndef RADIO_CONTROLLER_H
#define RADIO_CONTROLLER_H

#include "EventQueue.h"

#include <stdint.h>

// Enum for radio-specific events
enum class RadioEventId : uint8_t { RADIO_RX, RADIO_TX };

class RadioController {
public:
    RadioController(EventQueue *gaspettoQueue, const uint8_t writing_addr[5],
                    const uint8_t reading_addr[5]);
    void Init();
    void ProcessRadio();
    void SendEvent(Event evt);
    EventQueue *getRadioQueue();

private:
    uint8_t writing_addr[5];
    uint8_t reading_addr[5];
    EventQueue radioQueue;
    EventQueue *gaspettoQueue;
};

#endif // RADIO_CONTROLLER_H

#include "RadioController.h"

#include "Event.h"
// #include <RF24.h> // Uncomment and configure if using RF24 library

RadioController::RadioController(EventQueue *gaspettoQueue)
        : gaspettoCarQueue(gaspettoQueue)
{
    // Initialize NRF24L01 hardware here if needed
}

void RadioController::process_radio()
{
    // Example pseudo-code for receiving a command and posting an event
    // if (radio.available()) {
    //     uint8_t command;
    //     radio.read(&command, sizeof(command));
    //     // Post to radioQueue as RADIO_RX event
    //     Event evt(static_cast<EventId>(RadioEventId::RADIO_RX), static_cast<CommandId>(command));
    //     radioQueue.enqueue(evt);
    //     // Forward to GaspettoCar queue
    //     gaspettoCarQueue->enqueue(evt);
    // }
}

void RadioController::send_action(uint8_t action)
{
    // Example pseudo-code for sending the current action to an external receiver
    // radio.write(&action, sizeof(action));
    // Post to radioQueue as RADIO_TX event
    // Event evt(static_cast<EventId>(RadioEventId::RADIO_TX), static_cast<CommandId>(action));
    // radioQueue.enqueue(evt);
}

EventQueue *RadioController::getRadioQueue()
{
    return &radioQueue;
}

#include "Arduino.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "TimeredEventQueue.h"
#define NRF_IRQ_PIN PB0

IdleState idleState;
ProcessingState processingState;
EventQueue eventQueue;
TimeredEventQueue timeredEventQueue;

GaspettoCar gaspetto_car(&idleState, &processingState, &eventQueue, StateId::IDLE);

#ifndef ARDUINO
void enqueue_random_commands(const uint8_t num_events)
{
    srand(std::time(nullptr));

    for (uint8_t i = 0; i < num_events; ++i) {
        const CommandId command =
                static_cast<CommandId>(rand() % static_cast<int>(CommandId::MAX_COMMAND_ID));
        const Event event(EventId::NRF_IRQ, command); /* Random event*/
        gaspetto_car.postEvent(event);
    }
}
#endif

void ISR(void)
{
#ifndef ARDUINO
    Event evt = getEvent();
    gaspetto_car.postEvent(evt);
#endif
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Gaspetto Car Initialized");
    Serial.println("Starting up...\n");
    /* Initialize the GaspettoCar state machine. */
    gaspetto_car.Init();
#ifdef NRF_IRQ
    /* Set up ISR for NRF IRQ. */
    attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), ISR, RISING);
#else
    timeredEventQueue.scheduleEventDelayed(1000, Event(EventId::NRF_IRQ, CommandId::MOTOR_FORWARD));
    timeredEventQueue.scheduleEventDelayed(4000,
                                           Event(EventId::NRF_IRQ, CommandId::MOTOR_BACKWARD));
    timeredEventQueue.scheduleEventDelayed(10000, Event(EventId::NRF_IRQ, CommandId::MOTOR_RIGHT));
    timeredEventQueue.scheduleEventDelayed(6000, Event(EventId::NRF_IRQ, CommandId::MOTOR_STOP));
    timeredEventQueue.scheduleEventDelayed(2000, Event(EventId::NRF_IRQ, CommandId::MOTOR_LEFT));
    timeredEventQueue.scheduleEventDelayed(17000, Event(EventId::NRF_IRQ, CommandId::MOTOR_STOP));
#endif
}

void loop()
{
    gaspetto_car.processNextEvent();
    timeredEventQueue.processEvents(gaspetto_car);
}

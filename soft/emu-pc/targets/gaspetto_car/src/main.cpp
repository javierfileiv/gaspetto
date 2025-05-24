#include "Arduino.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "IdleState.h"
#include "MotorController.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "TimeredEventQueue.h"
#include "radio_controller/defs.h"

#include <cstdint>

// const int NRF_IRQ_PIN = PB0;
RF24 radio(CE_PIN, CSN_PIN);
EventQueue eventQueue;
IdleState idleState;
ProcessingState processingState;
TimeredEventQueue timeredEventQueue;
MotorController motorController;
RadioController radioControllerCar(radio, &eventQueue, gaspetto_box_pipe_name,
                                   gaspetto_car_pipe_name);
GaspettoCar gaspetto_car(&idleState, &processingState, &eventQueue, &motorController, nullptr);

void ISR(void)
{
#if !defined(ARDUINO) && defined(NRF_IRQ) && defined(USE_RADIO_CONTROLLER)
    Event evt = getEvent();
    gaspetto_car.postEvent(evt);
#endif
}

void setup()
{
    Serial.begin(115200);
#ifdef ARDUINO
    while (!Serial) {
        /* Wait for serial port to connect. Needed for native USB port only */
    }
#endif
    /* Initialize the motor controller. */
    motorController.set_isr_instance(&motorController);
    motorController.setPins(MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B, MOTOR_RIGHT_PIN_A,
                            MOTOR_RIGHT_PIN_B, SPEED_SENSOR_LEFT_PIN, SPEED_SENSOR_RIGHT_PIN);
    /* Initialize the GaspettoCar state machine. */
    gaspetto_car.Init();
#ifndef USE_RADIO_CONTROLLER
    timeredEventQueue.scheduleEventDelayed(1000, Event(EventId::ACTION, CommandId::MOTOR_FORWARD));
    timeredEventQueue.scheduleEventDelayed(4000, Event(EventId::ACTION, CommandId::MOTOR_BACKWARD));
    timeredEventQueue.scheduleEventDelayed(10000, Event(EventId::ACTION, CommandId::MOTOR_RIGHT));
    timeredEventQueue.scheduleEventDelayed(6000, Event(EventId::ACTION, CommandId::MOTOR_STOP));
    timeredEventQueue.scheduleEventDelayed(2000, Event(EventId::ACTION, CommandId::MOTOR_LEFT));
    timeredEventQueue.scheduleEventDelayed(17000, Event(EventId::ACTION, CommandId::MOTOR_STOP));
#else
#ifdef NRF_IRQ
    /* Set up ISR for NRF IRQ. */
    attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), ISR, RISING);
#endif /* NRF_IRQ */
#endif /* USE_RADIO_CONTROLLER */
}

void loop()
{
    radioControllerCar.ProcessRadio();
    gaspetto_car.processNextEvent();
    timeredEventQueue.processEvents(gaspetto_car);
}

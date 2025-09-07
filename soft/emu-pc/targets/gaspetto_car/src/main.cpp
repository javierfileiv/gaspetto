#include "Arduino.h"
#include "Context.h"
#include "EventQueue.h"
#include "GaspettoCar.h"
#include "IdleState.h"
#include "MovementController.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "TimeredEventQueue.h"
#include "config_radio.h"

#include <cstdint>

// const int NRF_IRQ_PIN = PB0;
RF24 radio(CE_PIN, CSN_PIN);
EventQueue eventQueue;
IdleState idleState;
ProcessingState processingState;
TimeredEventQueue timeredEventQueue;
MotorControl motorControl(MOTOR_LEFT_PIN_A, MOTOR_LEFT_PIN_B, MOTOR_RIGHT_PIN_A, MOTOR_RIGHT_PIN_B);
MovementController carMovementController(motorControl, SPEED_SENSOR_LEFT_PIN,
                                         SPEED_SENSOR_RIGHT_PIN);
RadioController radioControllerCar(radio, &eventQueue, gaspetto_box_pipe_name,
                                   gaspetto_car_pipe_name);
Context context = {
    &eventQueue,
    &carMovementController,
    &radioControllerCar,
    &timeredEventQueue,
    &idleState,
    &processingState,
    PWM_FREQ,
};
GaspettoCar gaspetto_car(context);

void ISR(void)
{
#if !defined(ARDUINO) && defined(NRF_IRQ) && defined(USE_RADIO_CONTROLLER)
    Event evt = getEvent();
    gaspetto_car.postEvent(evt);
#endif
}

void enter_low_power_mode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    Serial.println("Entering low-power mode...\n");
    SwitchToLowPowerMode();
#else
    /*  Implement low-power mode for Arduino. */
    /*  STM32 sleep modes or power-saving. */
    delay(100); /*  Simulate low-power sleep. */
#endif
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
    /* Initialize the GaspettoCar state machine. */
    gaspetto_car.setLowPowerModeCallback(enter_low_power_mode);
    gaspetto_car.init(StateId::IDLE);
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
#ifdef USE_RADIO_CONTROLLER
    radioControllerCar.ProcessRadio();
#else
    timeredEventQueue.processEvents(gaspetto_car);
#endif
    gaspetto_car.processNextEvent();
}

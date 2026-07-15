#include "Arduino.h"
#include "CarEvents.h"
#include "CarStates.h"
#include "Context.h"
#include "EventQueue.h"
#include "GCar.h"
#include "IMUOrientation.h"
#include "IdleState.h"
#include "Log.h"
#include "MotorControl.h"
#include "MovementController.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "RadioProtocol.h"
#include "TimeredEventQueue.h"
#include "config_radio.h"

#include <cstdint>

#ifndef ARDUINO
/* External functions defined to insert events in case of PC simulation. */
extern Event getEmulatedEvent(void);
#endif

/* const int NRF_IRQ_PIN = PB0; */
RF24 radio(NRF24_CE, NRF24_CSN);
EventQueue eventQueue;
IdleState idleState;
ProcessingState processingState;
IMUOrientation imu;
MotorControl motorControl(MOTOR_LEFT_BWD, MOTOR_LEFT_FWD, MOTOR_RIGHT_BWD, MOTOR_RIGHT_FWD);
MovementController carMovementController(motorControl, imu);
RadioController radioControllerCar(radio, &eventQueue, gbox_pipe_name, gcar_pipe_name);

TimeredEventQueue timeredEventQueue;

Context context = {
    &eventQueue, &carMovementController, &radioControllerCar, &timeredEventQueue,
    &idleState,  &processingState,       MOTOR_FREQ,
};
GCar gcar(context);
Log mainLog;

void ISR(void)
{
#ifndef ARDUINO
    Event evt = getEmulatedEvent();
    gcar.postEvent(evt);
#endif
}

void enter_low_power_mode()
{
#ifdef LOW_POWER_MODE
#ifndef ARDUINO
    mainLog.logln("Entering low-power mode...\n");
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
#ifdef GASPETTO_LOG_OVER_NRF24
    Log::attachNrf24(radio, gcar_log_pipe_name, gcar_pipe_name);
#endif
#ifdef ARDUINO
    while (!Serial) {
        /* Wait for serial port to connect. Needed for native USB port only */
    }
#endif
    mainLog.logln();
    mainLog.logln(F("GCar boot"));

    /* Initialize the GCar state machine. */
    gcar.setLowPowerModeCallback(enter_low_power_mode);
    mainLog.logln(F("GCar init begin"));
    gcar.init(StateId::IDLE);
    mainLog.logln(F("GCar init done"));

#ifdef USE_RADIO_CONTROLLER
    /* Set up telemetry callback to send IMU/PID data via radio */
    carMovementController.setTelemetryCallback(
            [](const TelemetryPacket &telemetry) { radioControllerCar.sendTelemetry(telemetry); });
#ifdef NRF_IRQ
    /* Set up ISR for NRF IRQ. */
    attachInterrupt(digitalPinToInterrupt(NRF_IRQ_PIN), ISR, RISING);
#endif /* NRF_IRQ */
#else /* USE_RADIO_CONTROLLER */
    timeredEventQueue.scheduleEventDelayed(1000, Event(EventId::ACTION, CommandId::MOTOR_FORWARD));
    timeredEventQueue.scheduleEventDelayed(6000, Event(EventId::ACTION, CommandId::MOTOR_STOP));
    timeredEventQueue.scheduleEventDelayed(9000, Event(EventId::ACTION, CommandId::MOTOR_BACKWARD));
    timeredEventQueue.scheduleEventDelayed(12000, Event(EventId::ACTION, CommandId::MOTOR_STOP));
    timeredEventQueue.scheduleEventDelayed(15000,
                                           Event(EventId::ACTION, CommandId::MOTOR_TURN_RIGHT));
    timeredEventQueue.scheduleEventDelayed(18000, Event(EventId::ACTION, CommandId::MOTOR_STOP));
    timeredEventQueue.scheduleEventDelayed(21000,
                                           Event(EventId::ACTION, CommandId::MOTOR_TURN_LEFT));
    timeredEventQueue.scheduleEventDelayed(24000, Event(EventId::ACTION, CommandId::MOTOR_STOP));
    mainLog.logln(F("Timered test sequence scheduled"));
#endif /* USE_RADIO_CONTROLLER */
}

void loop()
{
    gcar.work();
}

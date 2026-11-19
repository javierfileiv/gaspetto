#include "Arduino.h"
#include "Context.h"
#include "EventQueue.h"
#include "Events.h"
#include "FirmwareVersion.h"
#include "GCar.h"
#include "IMUOrientation.h"
#include "IdleState.h"
#include "Log.h"
#include "MotorControl.h"
#include "MovementController.h"
#include "OtaReceiver.h"
#include "ProcessingState.h"
#include "RF24.h"
#include "RadioController.h"
#include "RadioProtocol.h"
#include "States.h"
#include "TimeredEventQueue.h"
#include "WireVersion.h"
#include "config_radio.h"
#include "dfu_bootloader.h"

#include <cstdint>

#ifndef ARDUINO
/* External functions defined to insert events in case of PC simulation. */
extern Event getEmulatedEvent(void);
#endif

RF24 radio(NRF24_CE, NRF24_CSN);
EventQueue eventQueue;
IdleState idleState;
ProcessingState processingState;
IMUOrientation imu;
MotorControl motorControl(MOTOR_LEFT_BWD, MOTOR_LEFT_FWD, MOTOR_RIGHT_FWD, MOTOR_RIGHT_BWD);
MovementController carMovementController(motorControl, imu);
RadioController radioControllerCar(radio, &eventQueue, gbox_pipe_name, gcar_pipe_name);
OtaReceiver otaReceiver(radioControllerCar);

TimeredEventQueue timeredEventQueue;

Context context = {
    &eventQueue, &carMovementController, &radioControllerCar, &timeredEventQueue,
    &idleState,  &processingState,       MOTOR_FREQ,
};
GCar gcar(context);

/* The radio hands every OTA packet to the receiver; a plain function keeps the
 * radio controller free of any knowledge of flash or of the car */
void onOtaPacket(const OtaPacket &packet)
{
    /* An image transfer erases a staging sector for about two seconds, and the
     * loop is blocked while it does: the PID stops but the motors keep their
     * last duty, so a movement would carry on blind. Stopping here, before the
     * erase, and dropping the queue keeps the car still for the whole
     * transfer, which ends with a reset anyway */
    if (static_cast<OtaKind>(packet.kind) == OtaKind::Begin) {
        LOGLN(F("OTA: begin, stopping the car"));
        gcar.stopBothMotors();
        gcar.clearEventQueue();
    }

    otaReceiver.handle(packet);
}

/* The box animates the real end of a program, not its transmission: say so
 * once the queue has drained and the motors have stopped */
void onProgramDone()
{
    uint8_t payload[sizeof(CommandPacket)] = {};

    payload[0] = static_cast<uint8_t>(PacketTag::ProgramDone);
    radioControllerCar.sendBuffer(payload, sizeof(payload));
}

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
    LOGLN("Entering low-power mode...\n");
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
    /* Logs ride the normal channel, tagged: the destination and the
     * restore address are both the writing pipe (_box_), so the pipe
     * never moves and the receiver sorts packets by PacketTag */
    Log::attachNrf24(radio, gbox_pipe_name, gbox_pipe_name);
#endif
#ifdef ARDUINO
    while (!Serial) {
        /* Wait for serial port to connect. Needed for native USB port only */
    }
#endif
    LOGLN();
    LOGLN(F("GCar boot"));

    /* Initialize the GCar state machine. */
    gcar.setLowPowerModeCallback(enter_low_power_mode);
    /* The radio delivers a DFU request as an event: the jump itself is a
     * target concern, so main wires it and the state machine stays generic */
    gcar.setDfuCallback(enterDfuBootloader);
    gcar.setProgramDoneCallback(onProgramDone);
    radioControllerCar.setOtaHandler(onOtaPacket);
    LOGLN(F("GCar init begin"));
    gcar.init(StateId::IDLE);

    LOG(F("GCar firmware build "));
    LOG(FW_VERSION);
    LOG(F(" ("));
    LOG(FW_BUILD);
    LOG(F("), wire protocol v"));
    LOGLN(kWireProtocolVersion);
    LOGLN(F("GCar init done"));

#ifdef USE_RADIO_CONTROLLER
    /* Set up telemetry callback to send IMU/PID data via radio */
    carMovementController.setTelemetryCallback(
            [](const TelemetryPacket &telemetry) { radioControllerCar.sendTelemetry(telemetry); });
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
    LOGLN(F("Timered test sequence scheduled"));
#endif /* USE_RADIO_CONTROLLER */
}

void loop()
{
    gcar.work();
}

#include "CarEvents.h"
#include "GCar_ino.h"
#include "Serial.h"

#ifdef USE_RADIO_CONTROLLER
extern EventPacket pkt;
#else
extern Event event;
#endif

void gcar_input_switch(char ch)
{
    switch (ch) {
#ifdef USE_RADIO_CONTROLLER
    case 'W':
    case 'w':
        pkt.eventId = static_cast<uint8_t>(EventId::ACTION);
        pkt.payload = static_cast<uint8_t>(CommandId::MOTOR_FORWARD);
        radio.simulateReceivedPacket(0, &pkt, sizeof(pkt));
        break;
    case 'S':
    case 's':
        pkt.eventId = static_cast<uint8_t>(EventId::ACTION);
        pkt.payload = static_cast<uint8_t>(CommandId::MOTOR_BACKWARD);
        radio.simulateReceivedPacket(0, &pkt, sizeof(pkt));
        break;
    case 'A':
    case 'a':
        pkt.eventId = static_cast<uint8_t>(EventId::ACTION);
        pkt.payload = static_cast<uint8_t>(CommandId::MOTOR_TURN_LEFT);
        radio.simulateReceivedPacket(0, &pkt, sizeof(pkt));
        break;
    case 'D':
    case 'd':
        pkt.eventId = static_cast<uint8_t>(EventId::ACTION);
        pkt.payload = static_cast<uint8_t>(CommandId::MOTOR_TURN_RIGHT);
        radio.simulateReceivedPacket(0, &pkt, sizeof(pkt));
        break;
    case 'X':
    case 'x':
        pkt.eventId = static_cast<uint8_t>(EventId::ACTION);
        pkt.payload = static_cast<uint8_t>(CommandId::MOTOR_STOP);
        radio.simulateReceivedPacket(0, &pkt, sizeof(pkt));
        break;
#else
    case 'F':
    case 'f':
        event = Event(EventId::ACTION, CommandId::MOTOR_FORWARD);
        break;
    case 'b':
    case 'B':
        event = Event(EventId::ACTION, CommandId::MOTOR_BACKWARD);
        break;
    case 'l':
    case 'L':
        event = Event(EventId::ACTION, CommandId::MOTOR_TURN_LEFT);
        break;
    case 'r':
    case 'R':
        event = Event(EventId::ACTION, CommandId::MOTOR_TURN_RIGHT);
        break;
    case 's':
    case 'S':
        event = Event(EventId::ACTION, CommandId::MOTOR_STOP);
        break;
    case 'p':
    case 'P':
        event = Event(EventId::BUTTON_PRESSED, CommandId::NONE);
        break;
#endif
    }
}

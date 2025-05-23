#include "Event.h"
#include "GaspettoCar_ino.h"

extern Event event;

void nrf_sender_input_switch(char ch)
{
    event = Event(EventId::NONE, CommandId::NONE); /*  Reset event. */
    switch (ch) {
    case 'W':
    case 'w':
        event = Event(EventId::ACTION, CommandId::MOTOR_FORWARD);
        break;
    case 'S':
    case 's':
        event = Event(EventId::ACTION, CommandId::MOTOR_BACKWARD);
        break;
    case 'A':
    case 'a':
        event = Event(EventId::ACTION, CommandId::MOTOR_LEFT);
        break;
    case 'D':
    case 'd':
        event = Event(EventId::ACTION, CommandId::MOTOR_RIGHT);
        break;
    case 'X':
    case 'x':
        event = Event(EventId::ACTION, CommandId::MOTOR_STOP);
        break;
    }
}

#include "Event.h"
#include "Serial.h"

extern Event event;

void gaspetto_box_input_switch(char ch)
{
    switch (ch) {
    case 'p':
    case 'P':
        event = Event(EventId::BUTTON_PRESSED, CommandId::NONE);
        break;
    }
}

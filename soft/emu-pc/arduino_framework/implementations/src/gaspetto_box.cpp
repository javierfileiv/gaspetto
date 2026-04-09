#include "CarEvents.h"
#include "RF24.h"
#include "Serial.h"

extern Event event;
extern RF24 radio;

void gaspetto_box_input_switch(char ch)
{
    switch (ch) {
    case 'p':
    case 'P':
        event = Event(EventId::BUTTON_PRESSED, CommandId::NONE);
        break;
    case 'f':
    case 'F':
        radio.simulateFailedTransmission();
        Serial.println("Gaspetto Box: next RF transmission will fail.");
        break;
    }
}

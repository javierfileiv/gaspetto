#include "Arduino.h"
#include "Event.h"
#include "RF24.h"
#include "config_radio.h"

#ifdef ARDUINO_AVR_UNO
#define CE_PIN 9
#define CSN_PIN 10
#define HARDWARE_SERIAL Serial
#else
#define HARDWARE_SERIAL Serial1
#define CE_PIN PB15
#define CSN_PIN PA4
#endif

RF24 radio(CE_PIN, CSN_PIN);

void setup()
{
    Serial.begin(115200);
#ifdef ARDUINO
    while (!Serial) {
        delay(10);
    }
#endif
    Serial.println("NRF Event Sender - Sends commands over radio");
    if (!radio.begin()) {
        HARDWARE_SERIAL.println(F("radio hardware is not responding!!"));
        while (1) {
        } /* Hold in infinite loop. */
    }
    radio.setPALevel(PA_LEVEL);
    radio.setDataRate(DATA_RATE); // better reliability
    radio.setAddressWidth(5);
    radio.setPayloadSize(Event::packetSize());
    radio.openWritingPipe(gaspetto_car_pipe_name);
    radio.openReadingPipe(1, gaspetto_box_pipe_name);
    radio.powerUp();
    radio.printDetails();
    radio.printPrettyDetails();
    radio.startListening();
}

void sendEvent(CommandId cmd)
{
    Event evt(EventId::ACTION, cmd);
    EventPacket pkt;
    evt.toPacket(pkt);
    radio.stopListening();
    radio.write(&pkt, sizeof(pkt));
    HARDWARE_SERIAL.print("Sent command: ");
    HARDWARE_SERIAL.println(Event::commandIdToString(cmd));
    radio.startListening();
}

void loop()
{
    if (HARDWARE_SERIAL.available()) {
        char ch = HARDWARE_SERIAL.read();
        switch (ch) {
        case 'w':
        case 'W':
            sendEvent(CommandId::MOTOR_FORWARD);
            break;
        case 's':
        case 'S':
            sendEvent(CommandId::MOTOR_BACKWARD);
            break;
        case 'a':
        case 'A':
            sendEvent(CommandId::MOTOR_LEFT);
            break;
        case 'd':
        case 'D':
            sendEvent(CommandId::MOTOR_RIGHT);
            break;
        case 'x':
        case 'X':
            sendEvent(CommandId::MOTOR_STOP);
            break;
        }
    }
    if (radio.available()) {
        uint8_t pkt;
        radio.read(&pkt, sizeof(pkt));
        HARDWARE_SERIAL.print("Received event: cmd= ");
        HARDWARE_SERIAL.println(pkt);
    }
    delay(10);
}

#include "Arduino.h"
#include "Event.h"
#include "RF24.h"

#include <Arduino.h>

#ifdef ARDUINO_AVR_UNO
#define CE_PIN 9
#define CSN_PIN 10
#else
#define CE_PIN PB_15
#define CSN_PIN PIN_A4
#endif

const uint8_t gaspetto_box_pipe_name[] = "_box_";
const uint8_t gaspetto_car_pipe_name[] = "_car_";
RF24 radio(CE_PIN, CSN_PIN);

void setup()
{
    Serial.begin(115200);
#ifdef ARDUINO
    while (!Serial) {
        delay(10);
    }
#endif
    Serial.println("NRF Event Sender (Arduino-style, PC/Arduino compatible)");
    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {
        } /* Hold in infinite loop. */
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
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
    Serial.print("Sent command: ");
    Serial.println(commandIdToString(cmd));
    radio.startListening();
}

void loop()
{
    if (Serial.available()) {
        char ch = Serial.read();
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
    delay(100);
}

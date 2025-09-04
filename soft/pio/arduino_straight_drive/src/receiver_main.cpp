// Remote receiver: forwards serial commands via NRF24 and prints telemetry in same DATA format
#include <Arduino.h>
#include <RF24.h>
#include <RadioProtocol.h>
#include <SPI.h>

#ifndef RADIO_CE_PIN
#define RADIO_CE_PIN 9
#endif
#ifndef RADIO_CSN_PIN
#define RADIO_CSN_PIN 10
#endif

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
bool radioOk = false;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }
    Serial.println(F("Remote receiver starting"));
    if (radio.begin())
    {
        radio.setAddressWidth(5);
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_250KBPS);
        radio.setChannel(108);
        radio.setRetries(5, 5);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();
        radio.openReadingPipe(1, RADIO_ADDR_TLM); // listen to telemetry from vehicle
        radio.startListening();
        radioOk = true;
        Serial.println(F("Radio OK"));
    }
    else
    {
        Serial.println(F("Radio init FAILED"));
    }
    Serial.println(F("Receiver ready (command passthrough mode)."));
}

void loop()
{
    // User command input only (no telemetry print to keep serial quiet)
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() && radioOk) {
            CommandPacket cp{};
            line.toCharArray(cp.text, sizeof(cp.text));
            radio.stopListening();
            radio.openWritingPipe(RADIO_ADDR_CMD);
            bool ackOk = radio.write(&cp, sizeof(cp)); // try with ACK
            if (!ackOk) {
                bool noAckOk = radio.write(&cp, sizeof(cp), true); // fallback
                Serial.print(F("TX:")); Serial.print(line); Serial.print(F(":"));
                if (noAckOk) Serial.println(F("FALLBACK")); else Serial.println(F("FAIL"));
            } else {
                Serial.print(F("TX:")); Serial.print(line); Serial.println(F(":OK"));
            }
            radio.startListening();
        }
    }
}

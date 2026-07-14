#include "Arduino.h"
#include "RF24.h"

#include <cstring>

static constexpr uint32_t CE_PIN = PB_15;
static constexpr uint32_t CSN_PIN = PA_4;
static const uint8_t gaspetto_box_log_pipe_name[] = "_logb";
static const uint8_t gcar_log_pipe_name[] = "_logc";

RF24 radio(CE_PIN, CSN_PIN);

static void printHelp()
{
    Serial.println("\\n=== NRF Log Receiver ===");
    Serial.println("Listening on pipes:");
    Serial.println("  Pipe 1: _logb (GaspettoBox logs)");
    Serial.println("  Pipe 2: _logc (GCar logs)");
    Serial.println("Messages are auto-fragmented. Newline ends a log line.");
    Serial.println("========================\\n");
}

void setup()
{
    Serial.begin(115200);
#ifdef ARDUINO
    const unsigned long deadline = millis() + 1500;
    while (!Serial && millis() < deadline) {
        delay(10);
    }
#endif

    Serial.println("NRF Log Receiver boot");

    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {
            delay(1000);
        }
    }

    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setAddressWidth(5);
    radio.setPayloadSize(32);
    radio.openReadingPipe(1, gaspetto_box_log_pipe_name);
    radio.openReadingPipe(2, gcar_log_pipe_name);
    radio.startListening();

    radio.printDetails();
    radio.printPrettyDetails();
    printHelp();
}

void loop()
{
    static uint8_t boxMessageBuffer[512];
    static uint16_t boxMessageLen = 0;
    static uint8_t carMessageBuffer[512];
    static uint16_t carMessageLen = 0;

    uint8_t pipe = 0;
    if (!radio.available(&pipe)) {
        delay(5);
        return;
    }

    uint8_t payload[32] = { 0 };
    radio.read(&payload, sizeof(payload));

    uint8_t flags = payload[0];
    uint8_t dataLen = flags & 0x7F; // Extract data length (bits 6-0)
    bool hasMore = (flags & 0x80) != 0; // Check continuation flag (bit 7)

    // Route to appropriate buffer based on pipe
    uint8_t *messageBuffer = (pipe == 1) ? boxMessageBuffer : carMessageBuffer;
    uint16_t *messageLen = (pipe == 1) ? &boxMessageLen : &carMessageLen;
    const char *source = (pipe == 1) ? "[BOX]" : "[CAR]";

    // Append data to message buffer
    if (*messageLen + dataLen <= 512) {
        std::memcpy(&messageBuffer[*messageLen], &payload[1], dataLen);
        *messageLen += dataLen;
    }

    // If this is the final packet (no continuation), output the complete message
    if (!hasMore) {
        Serial.print(source);
        Serial.print(" ");
        for (uint16_t i = 0; i < *messageLen; ++i) {
            Serial.write(messageBuffer[i]);
        }
        *messageLen = 0;
    }
}

#include "Arduino.h"
#include "CommandPacket.h"
#include "PacketTag.h"
#include "RF24.h"
#include "RadioProtocol.h"
#include "config_radio.h"

#include <cstring>

/* CE and CSN of this receiver's own header: not the GBOX pins of
 * config_radio.h, which comes in for the two channel addresses only.
 * Its "#warning: no radio pins" is expected here for the same reason */
static constexpr uint32_t CE_PIN = PB_15;
static constexpr uint32_t CSN_PIN = PA_4;

RF24 radio(CE_PIN, CSN_PIN);

static void printHelp()
{
    Serial.println("\n=== NRF Log Receiver ===");
    Serial.println("Both channels pass through this rig:");
    Serial.println("  Pipe 1: _car_ (written by the box: programs and its logs)");
    Serial.println("  Pipe 0: _box_ (written by the car: telemetry and its logs)");
    Serial.println("Log lines are reassembled; a newline ends a line.");
    Serial.println("Programs print their commands, telemetry prints yaw and state,");
    Serial.println("events and unknown tags are skipped.");
    Serial.println("========================\n");
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
    /* Both channels at once: pipe 1 carries what the box writes (its logs
     * and its programs) and pipe 0 what the car writes (telemetry and
     * logs). openReadingPipe(0) is what keeps pipe 0 in RX across
     * startListening(), which closes it otherwise */
    radio.openReadingPipe(1, gcar_pipe_name);
    radio.openReadingPipe(0, gbox_pipe_name);
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

    /* Programs on _car_ and telemetry on _box_ share these channels: byte 0
     * holds a PacketTag for the tagged kinds and the command count for a
     * program, so one test per case says what arrived */
    const char *source = (pipe == 1) ? "[BOX]" : "[CAR]";
    const uint8_t kind = payload[0];

    if (kind == static_cast<uint8_t>(PacketTag::Telemetry)) {
        TelemetryPacket telemetry;

        std::memcpy(&telemetry, payload, sizeof(telemetry));
        Serial.print(source);
        Serial.print(" telemetry yaw=");
        Serial.print(telemetry.yaw, 1);
        Serial.print(" tgt=");
        Serial.print(telemetry.targetYaw, 1);
        Serial.print(" state=");
        Serial.println(static_cast<int>(telemetry.state));
        return;
    }

    if (kind > 0 && kind <= BOX_MAX_PAYLOAD_COMMANDS) {
        /* A program: the pieces the box validated */
        Serial.print(source);
        Serial.print(" program:");
        for (uint8_t i = 0; i < kind; ++i) {
            Serial.print(' ');
            Serial.print(commandIdToString(static_cast<CommandId>(payload[1 + i])));
        }
        Serial.println();
        return;
    }

    if (kind != static_cast<uint8_t>(PacketTag::Log)) {
        /* Events and unknown tags: nothing to print here */
        return;
    }

    uint8_t flags = payload[1];
    uint8_t dataLen = flags & 0x7F; // Extract data length (bits 6-0)
    bool hasMore = (flags & 0x80) != 0; // Check continuation flag (bit 7)

    // Route to appropriate buffer based on pipe
    uint8_t *messageBuffer = (pipe == 1) ? boxMessageBuffer : carMessageBuffer;
    uint16_t *messageLen = (pipe == 1) ? &boxMessageLen : &carMessageLen;

    // Append data to message buffer
    if (*messageLen + dataLen <= 512) {
        std::memcpy(&messageBuffer[*messageLen], &payload[2], dataLen);
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

#include "Arduino.h"
#include "CarEvents.h"
#include "CommandPacket.h"
#include "RF24.h"
#include "config_radio.h"

#include <Arduino.h>

RF24 radio(NRF24_CE, NRF24_CSN);

/* Forward declarations. */
void printHelp();

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
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {
        } /* Hold in infinite loop. */
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setAddressWidth(5);
    /* Support CommandPacket (32 bytes) for multi-command programs. */
    radio.setPayloadSize(sizeof(CommandPacket));
    radio.openWritingPipe(gcar_pipe_name);
    radio.openReadingPipe(1, gbox_pipe_name);
    radio.openReadingPipe(2, gcar_telemetry_pipe_name);
    radio.powerUp();
    radio.printDetails();
    radio.printPrettyDetails();
    radio.startListening();

    printHelp();
}

/* Program builder and queue. */
CommandPacket currentProgram{};

void sendCommand(CommandId cmd)
{
    /* Build a single-command program and send it. */
    CommandPacket pkt{};
    pkt.count = 1;
    pkt.commands[0] = static_cast<uint8_t>(cmd);

    radio.stopListening();
    radio.write(&pkt, sizeof(pkt));
    radio.startListening();

    Serial.print("Sent command: ");
    Serial.println(commandIdToString(cmd));
}

void addCommandToProgram(CommandId cmd)
{
    if (currentProgram.count < BOX_MAX_PAYLOAD_COMMANDS) {
        currentProgram.commands[currentProgram.count++] = static_cast<uint8_t>(cmd);
        Serial.print("Added to program: ");
        Serial.print(commandIdToString(cmd));
        Serial.print(" (count: ");
        Serial.print(currentProgram.count);
        Serial.println(")");
    } else {
        Serial.println("Program is full!");
    }
}

void sendProgram()
{
    if (currentProgram.count == 0) {
        Serial.println("Program is empty!");
        return;
    }

    radio.stopListening();
    radio.write(&currentProgram, sizeof(currentProgram));
    radio.startListening();

    Serial.print("Sent program with ");
    Serial.print(currentProgram.count);
    Serial.println(" commands");

    currentProgram = {}; /* Reset program. */
}

void clearProgram()
{
    currentProgram = {};
    Serial.println("Program cleared");
}

void loop()
{
    if (Serial.available()) {
        char ch = Serial.read();
        switch (ch) {
        /* Single commands (send immediately). */
        case 'w':
            sendCommand(CommandId::MOTOR_FORWARD);
            break;
        case 's':
            sendCommand(CommandId::MOTOR_BACKWARD);
            break;
        case 'a':
            sendCommand(CommandId::MOTOR_TURN_LEFT);
            break;
        case 'd':
            sendCommand(CommandId::MOTOR_TURN_RIGHT);
            break;
        case 'x':
            sendCommand(CommandId::MOTOR_STOP);
            break;

        /* Program commands (add to program). */
        case 'W':
            addCommandToProgram(CommandId::MOTOR_FORWARD);
            break;
        case 'S':
            addCommandToProgram(CommandId::MOTOR_BACKWARD);
            break;
        case 'A':
            addCommandToProgram(CommandId::MOTOR_TURN_LEFT);
            break;
        case 'D':
            addCommandToProgram(CommandId::MOTOR_TURN_RIGHT);
            break;
        case 'X':
            addCommandToProgram(CommandId::MOTOR_STOP);
            break;

        /* Program control. */
        case '*':
            sendProgram();
            break;
        case '-':
            clearProgram();
            break;

        case '?':
        case 'h':
        case 'H':
            printHelp();
            break;
        }
    }
    delay(100);
}

void printHelp()
{
    Serial.println("\n=== NRF Sender - CommandPacket Mode ===");
    Serial.println("Single commands (send immediately):");
    Serial.println("  w/W: Forward");
    Serial.println("  s/S: Backward");
    Serial.println("  a/A: Turn Left");
    Serial.println("  d/D: Turn Right");
    Serial.println("  x/X: Stop");
    Serial.println("\nProgram mode:");
    Serial.println("  Lowercase (w,s,a,d,x): Send single command");
    Serial.println("  Uppercase (W,S,A,D,X): Add to program queue");
    Serial.println("  *: Send program");
    Serial.println("  -: Clear program");
    Serial.println("  ?: Print this help");
    Serial.println("========================================\n");
}

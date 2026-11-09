#include "Arduino.h"
#include "CommandPacket.h"
#include "Events.h"
#include "Log.h"
#include "RF24.h"
#include "RadioProtocol.h"
#include "config_radio.h"

#include <Arduino.h>
#include <avr/wdt.h>

/* Mode 1 emulates the car: it listens for the program the Box sends on
 * _car_ and answers with telemetry on _box_, where the box listens. Mode 2
 * emulates the box: it sends commands on _car_ and listens for telemetry
 * on _box_. The car is the default because the bench test drives the box
 * side. */
constexpr uint8_t kModeCar = 1;
constexpr uint8_t kModeBox = 2;

/* One telemetry packet per second in car mode, so the link can be watched
 * without touching the keyboard */
constexpr uint32_t kTelemetryPeriodMs = 1000;

RF24 radio(NRF24_CE, NRF24_CSN);

uint8_t gMode = kModeCar;
uint32_t gLastTelemetryMs = 0;
uint16_t gTelemetryTick = 0;

/* One simulated car log line every kLogPeriodMs, offset from the telemetry
 * period so the fragments and the telemetry never land on the same beat */
constexpr uint32_t kLogPeriodMs = 430;
uint32_t gLastLogMs = 0;
uint16_t gLogTick = 0;

/* Program builder and queue, mode 2 only */
CommandPacket currentProgram{};

/* Forward declarations */
void printHelp();
void setMode(uint8_t mode);
void sendTelemetry();
void sendCarLog();
void drainRadio();
void displayTelemetry(const TelemetryPacket &t);
void displayCommandPacket(const CommandPacket &pkt);

void setup()
{
    Serial.begin(115200);
#ifdef ARDUINO
    while (!Serial) {
        delay(10);
    }
#endif
    Serial.println("NrfLink - Gaspetto radio link tool");
    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {
        } /* Hold in infinite loop. */
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setAddressWidth(5);
    /* Both payloads are 32 bytes, CommandPacket sets the size for both */
    radio.setPayloadSize(sizeof(CommandPacket));
    radio.powerUp();

    setMode(kModeCar);

    radio.printDetails();
    radio.printPrettyDetails();
    printHelp();
}

/* RF24 v1.6 has no address getter: this prints the constants just written
 * into the chip, which is what two rigs compare when they do not hear each
 * other. printDetails() reaches nothing on this board (RF24 writes it with
 * printf_P, which is not wired to the USB port), hence Serial.print here */
static void printAddress(const uint8_t *addr)
{
    for (uint8_t i = 0; i < 5; ++i) {
        if (addr[i] < 0x10) {
            Serial.print('0');
        }
        Serial.print(addr[i], HEX);
        if (i < 4) {
            Serial.print(' ');
        }
    }
}

/* openWritingPipe() writes TX_ADDR and RX_ADDR_P0 together, which is what
 * the auto-acknowledge needs, so the two modes mirror each other: whoever
 * transmits owns pipe 0 and reads the other address on pipe 1. Both modes
 * therefore live on pipes 0 and 1 only: RF24 gives pipes 2-5 a single
 * address byte and takes the other four from pipe 1, and every Gaspetto
 * address starts with '_', so pipe 2 would only mirror pipe 1 */
void setMode(uint8_t mode)
{
    gMode = mode;

    if (gMode == kModeCar) {
        radio.openWritingPipe(gbox_pipe_name); /* pipe 0 = _box_, telemetry rides it */
        radio.openReadingPipe(1, gcar_pipe_name); /* programs */
    } else {
        radio.openWritingPipe(gcar_pipe_name); /* pipe 0 = _car_ */
        radio.openReadingPipe(1, gbox_pipe_name); /* telemetry from the car */
    }
    /* A subject's logs ride its own channel: the destination and the
     * restore address are the same, so the pipe never moves and Log.h
     * only has to tag the fragments it produces */
    const uint8_t *logAddr = (gMode == kModeCar) ? gbox_pipe_name : gcar_pipe_name;
    Log::attachNrf24(radio, logAddr, logAddr);

    radio.startListening();

    /* Both addresses on one line: this is the half of the link the bench
     * prints too, and matching the two lines is the whole diagnosis */
    Serial.print("[mode] ");
    Serial.print(gMode == kModeCar ? "1 = car emulation" : "2 = box/sender");
    Serial.print(", tx ");
    printAddress(gMode == kModeCar ? gbox_pipe_name : gcar_pipe_name);
    Serial.print(", rx(pipe1) ");
    printAddress(gMode == kModeCar ? gcar_pipe_name : gbox_pipe_name);
    Serial.println();
}

/* Telemetry values move every tick so a dead link is visible as a frozen
 * readout rather than as a plausible constant */
void sendTelemetry()
{
    TelemetryPacket t{};
    bool sent;

    /* The tag is what tells the box a telemetry apart from a program:
     * without it byte 0 would read as a command count */
    t.tag = static_cast<uint8_t>(PacketTag::Telemetry);
    t.targetYaw = static_cast<float>((gTelemetryTick * 7U) % 360U);
    t.yaw = static_cast<float>((gTelemetryTick * 5U) % 360U);
    t.imuOk = 1;
    t.state = static_cast<uint8_t>(gTelemetryTick % 3U);
    t.speed = static_cast<uint8_t>(50U + (gTelemetryTick % 50U));
    t.trim = static_cast<int8_t>((gTelemetryTick % 201U) - 100U);
    t.kp = 0.5f;
    t.ki = 0.1f;
    t.kd = 0.05f;
    t.pwmFreq = 1000.0f;

    radio.stopListening();
    sent = radio.write(&t, sizeof(t));
    radio.startListening();

    Serial.print("[tx] telemetry #");
    Serial.print(gTelemetryTick);
    Serial.println(sent ? " sent" : " FAILED (no acknowledge)");
    ++gTelemetryTick;
}

/* A simulated car log line: same framework (Log.h) and same channel as the
 * production GBox, content that moves every tick so a listener sees the
 * fragments interleave with the telemetry instead of a constant */
void sendCarLog()
{
    static const char *const kTopics[] = { "imu", "pid", "motor", "state", "battery" };

    ++gLogTick;
    LOG(F("car-sim ["));
    LOG(kTopics[gLogTick % 5U]);
    LOG(F("] tick="));
    LOG(gLogTick);
    LOG(F(" ms="));
    LOG(millis());
    LOG(F(" yaw="));
    LOG(static_cast<float>((gLogTick * 13U) % 360U), 1);
    LOG(F(" speed="));
    LOGLN(gLogTick % 60U);
}

void displayCommandPacket(const CommandPacket &pkt)
{
    uint8_t i;

    Serial.print("[rx] CommandPacket count=");
    Serial.print(pkt.count);
    for (i = 0; i < pkt.count && i < BOX_MAX_PAYLOAD_COMMANDS; ++i) {
        Serial.print(' ');
        Serial.print(commandIdToString(static_cast<CommandId>(pkt.commands[i])));
    }
    Serial.println();
}

void sendCommand(CommandId cmd)
{
    CommandPacket pkt{};
    bool sent;

    pkt.count = 1;
    pkt.commands[0] = static_cast<uint8_t>(cmd);

    radio.stopListening();
    sent = radio.write(&pkt, sizeof(pkt));
    radio.startListening();

    Serial.print(sent ? "Sent command: " : "FAILED to send: ");
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
    bool sent;

    if (currentProgram.count == 0) {
        Serial.println("Program is empty!");
        return;
    }

    radio.stopListening();
    sent = radio.write(&currentProgram, sizeof(currentProgram));
    radio.startListening();

    Serial.print(sent ? "Sent program with " : "FAILED to send program with ");
    Serial.print(currentProgram.count);
    Serial.println(" commands");

    currentProgram = {}; /* Reset program. */
}

void clearProgram()
{
    currentProgram = {};
    Serial.println("Program cleared");
}

/* The word "reset" typed in a row reboots the board. The watchdog is the
 * only full reset available on AVR: jumping to 0 would restart the code but
 * leave the radio and the peripherals in whatever state they were in. The
 * function reports whether the character was eaten by the sequence, so a
 * key that also starts the word (the "s" of reset) is not mistaken for a
 * command while the word is being typed */
bool consumeResetWord(char ch)
{
    static const char kResetWord[] = "reset";
    static uint8_t seen = 0;

    if (ch == kResetWord[seen]) {
        ++seen;
        if (seen == sizeof(kResetWord) - 1U) {
            Serial.println("Resetting the board...");
            Serial.flush();
            wdt_enable(WDTO_15MS);
            while (true) {
            }
        }
        return true;
    }

    /* Mismatch: restart on the first letter if this character starts the
     * word, so "rr" still counts as the beginning of "reset" */
    seen = (ch == kResetWord[0]) ? 1 : 0;
    return seen == 1;
}

void drainRadio()
{
    uint8_t pipe;

    while (radio.available(&pipe)) {
        /* pipe 1 carries the other side's payload: a program in car mode,
         * telemetry in box mode. pipe 0 is the address this mode transmits
         * to, so whatever lands on it is traffic meant for the other
         * device and is dropped */
        if (pipe == 1) {
            if (gMode == kModeCar) {
                CommandPacket cmd;

                radio.read(&cmd, sizeof(cmd));
                displayCommandPacket(cmd);
            } else {
                TelemetryPacket tlm;

                radio.read(&tlm, sizeof(tlm));
                displayTelemetry(tlm);
            }
        } else {
            uint8_t sink[sizeof(CommandPacket)];

            /* Not a payload this mode interprets: drop it so the FIFO
             * cannot fill up and stall the radio */
            radio.read(sink, sizeof(sink));
        }
    }
}

void handleKey(char ch)
{
    if (consumeResetWord(ch)) {
        return;
    }

    switch (ch) {
    case '1':
        setMode(kModeCar);
        break;
    case '2':
        setMode(kModeBox);
        break;
    case '?':
    case 'h':
    case 'H':
        printHelp();
        break;
    default:
        /* The command keys belong to the box side only: the car never
         * transmits motor commands, it answers with telemetry */
        if (gMode != kModeBox) {
            break;
        }
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
        }
        break;
    }
}

void loop()
{
    drainRadio();

    if (gMode == kModeCar && (millis() - gLastTelemetryMs) >= kTelemetryPeriodMs) {
        gLastTelemetryMs = millis();
        sendTelemetry();
    }

    if (gMode == kModeCar && (millis() - gLastLogMs) >= kLogPeriodMs) {
        gLastLogMs = millis();
        sendCarLog();
    }

    if (Serial.available()) {
        handleKey(static_cast<char>(Serial.read()));
    }

    delay(100);
}

void printHelp()
{
    Serial.println("\n=== NrfLink - Gaspetto radio link tool ===");
    Serial.println("Modes:");
    Serial.println("  1: car emulation (default) - reads the program the Box sends");
    Serial.println("     on _car_ and answers with telemetry and logs on _box_");
    Serial.println("  2: box/sender - sends commands on _car_, reads telemetry on _box_");
    Serial.println("  ?: this help");
    Serial.println("  reset: type the five letters in a row to reboot the MCU");
    Serial.println("\nMode 2 keys:");
    Serial.println("  w/s/a/d/x: send a single command");
    Serial.println("  W/S/A/D/X: add to the program queue");
    Serial.println("  *: send the queue, -: clear the queue");
    Serial.println("========================================\n");
}

void displayTelemetry(const TelemetryPacket &t)
{
    Serial.print(F("TLM yaw="));
    Serial.print(t.yaw, 1);
    Serial.print(F(" tgt="));
    Serial.print(t.targetYaw, 1);
    Serial.print(F(" err="));
    Serial.print(t.targetYaw - t.yaw, 2); /* err left the packet: the tag took its byte */
    Serial.print(F(" imu state="));
    Serial.print(t.imuOk);
    Serial.print(F(" kp="));
    Serial.print(t.kp, 2);
    Serial.print(F(" ki="));
    Serial.print(t.ki, 4);
    Serial.print(F(" kd="));
    Serial.print(t.kd, 4);
    Serial.print(F(" freq="));
    Serial.print(t.pwmFreq, 0);
    Serial.println(F("Hz"));
}

#include "Arduino.h"
#include "CommandPacket.h"
#include "Events.h"
#include "Log.h"
#include "OtaProtocol.h"
#include "RF24.h"
#include "RadioProtocol.h"
#include "config_radio.h"

#include <Arduino.h>
#include <avr/wdt.h>
#include <string.h>

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

/* A line is printed as its fragments arrive. Buffering a whole line would cap
 * its length, and the longest lines are exactly the ones worth reading: the
 * "Ignoring new motor command while already processing a movement" notice is
 * 78 bytes. One flag per source says whether the prefix was already printed */
uint8_t gLogLineStarted[2] = { 0, 0 };

/* Forward declarations */
void printHelp();
void setMode(uint8_t mode);
void sendTelemetry();
void sendCarLog();
void sendDfuRequest();
void drainRadio();
void runOtaStream();
void runOtaSelfTest();
void drainLogFragment(uint8_t pipe, const uint8_t *payload);
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
    /* The chip answered begin(): log fragments can now ride the air instead
     * of the console */
    Log::markNrf24Ready();

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

/* The car takes a tagged event as a device command: no motor command byte is
 * spent on it and no program can produce it. The payload is written at the
 * full 32 byte width so the receiver never depends on the chip padding the
 * tail of a short write */
void sendDfuRequest()
{
    uint8_t payload[sizeof(CommandPacket)] = { 0 };
    bool sent;

    payload[0] = static_cast<uint8_t>(PacketTag::Event);
    payload[1] = static_cast<uint8_t>(EventId::DFU_REQUEST);
    payload[2] = static_cast<uint8_t>(CommandId::NONE);

    /* Car emulation transmits on _box_, so the request would never reach the
     * car: the box mode owns the _car_ writing pipe */
    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    radio.stopListening();
    sent = radio.write(payload, sizeof(payload));
    radio.startListening();

    Serial.print(sent ? F("DFU request ACKed by the car") : F("DFU request FAILED (no ACK)"));
    Serial.println(F(", the board should re-enumerate in about a second"));
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

/* One log line per source: fragments are printed as they arrive, the prefix
 * only on the first one, and a newline once the continuation bit clears */
void drainLogFragment(uint8_t pipe, const uint8_t *payload)
{
    const uint8_t flags = payload[1];
    const uint8_t dataLen = flags & 0x7FU;
    const bool hasMore = (flags & 0x80U) != 0U;
    const uint8_t index = pipe & 1U;

    if (gLogLineStarted[index] == 0) {
        /* Pipe 1 is the other side's channel, and which device that is follows
         * the mode: car emulation reads the box, box emulation reads the car */
        Serial.print(gMode == kModeCar ? "[BOX] " : "[CAR] ");
        gLogLineStarted[index] = 1;
    }

    for (uint8_t i = 0; i < dataLen; ++i) {
        Serial.write(payload[2 + i]);
    }

    if (!hasMore) {
        if (dataLen == 0 || payload[2 + dataLen - 1] != '\n') {
            Serial.println();
        }
        gLogLineStarted[index] = 0;
    }
}

void drainRadio()
{
    uint8_t pipe;

    while (radio.available(&pipe)) {
        uint8_t payload[sizeof(CommandPacket)] = { 0 };

        radio.read(payload, sizeof(payload));

        /* pipe 1 carries the other side's payload: a program in car mode,
         * telemetry and logs in box mode. pipe 0 is the address this mode
         * transmits to, so whatever lands on it is traffic meant for the
         * other device and is dropped */
        if (pipe != 1) {
            continue;
        }

        /* Byte 0 says what arrived: a tag for a log, telemetry or event, and
         * the command count for a program. One comparison per case, instead
         * of assuming the payload matches the mode */
        if (payload[0] == static_cast<uint8_t>(PacketTag::Log)) {
            drainLogFragment(pipe, payload);
        } else if (gMode == kModeCar) {
            CommandPacket cmd;

            memcpy(&cmd, payload, sizeof(cmd));
            displayCommandPacket(cmd);
        } else if (payload[0] == static_cast<uint8_t>(PacketTag::Telemetry)) {
            TelemetryPacket tlm;

            memcpy(&tlm, payload, sizeof(tlm));
            displayTelemetry(tlm);
        } else {
            Serial.print("[rx] ignored packet, tag=0x");
            Serial.println(payload[0], HEX);
        }
    }
}

/* ========================================== */
/* OTA relay                                  */
/* ========================================== */
/* The PC streams a binary image on this port: a 0x01 start byte, the image
 * length and its CRC as two little-endian words, then the image itself. Every
 * 28-byte chunk becomes one Data packet, and one status byte goes back to the
 * PC so it never outruns the radio */
constexpr uint8_t kOtaStart = 0x01;
/* Control codes, not letters: NrfLink prints hex addresses and log lines
 * on the same port, and a plain 'F' would be read as a refusal */
constexpr uint8_t kOtaAck = 0x06;
constexpr uint8_t kOtaNak = 0x15;
constexpr uint16_t kOtaTimeoutMs = 2000;

bool readExact(uint8_t *buffer, uint8_t count, uint16_t timeoutMs)
{
    uint8_t got = 0;
    const uint32_t start = millis();

    while (got < count) {
        if (Serial.available() > 0) {
            buffer[got++] = static_cast<uint8_t>(Serial.read());
            continue;
        }
        if (millis() - start > timeoutMs) {
            return false;
        }
    }

    return true;
}

/* After a failed transfer the PC may still be sending: eat what is left so a
 * 0x01 inside the image cannot start a bogus session from the main loop */
void drainSerial(uint16_t ms)
{
    const uint32_t start = millis();

    while (millis() - start < ms) {
        while (Serial.available() > 0) {
            Serial.read();
        }
    }
}

void writeLe32(uint8_t *at, uint32_t value)
{
    at[0] = static_cast<uint8_t>(value);
    at[1] = static_cast<uint8_t>(value >> 8);
    at[2] = static_cast<uint8_t>(value >> 16);
    at[3] = static_cast<uint8_t>(value >> 24);
}

bool sendOtaPacket(OtaKind kind, uint16_t seq, const uint8_t *data, uint8_t length)
{
    OtaPacket packet = {};

    packet.tag = static_cast<uint8_t>(PacketTag::Ota);
    packet.kind = static_cast<uint8_t>(kind);
    packet.seq = seq;

    if (data != nullptr && length > 0) {
        memcpy(packet.data, data, length);
    }

    radio.stopListening();
    const bool sent = radio.write(&packet, sizeof(packet));
    radio.startListening();
    return sent;
}

bool sendOtaBegin(uint32_t length, uint32_t crc)
{
    uint8_t header[10] = {};

    header[0] = kWireProtocolVersion;
    writeLe32(&header[2], length);
    writeLe32(&header[6], crc);
    return sendOtaPacket(OtaKind::Begin, 0, header, sizeof(header));
}

enum class OtaWait : uint8_t { Ok, Error, Timeout };

/* Wait for the car's application-level answer. The radio acknowledgement only
 * says a packet reached the chip; this one says the image bytes were written */
OtaWait waitOtaResponse(uint16_t seq, uint16_t timeoutMs, uint8_t &errorCode)
{
    const uint32_t start = millis();

    while (millis() - start < timeoutMs) {
        uint8_t pipe;
        uint8_t buffer[sizeof(OtaPacket)];
        OtaPacket packet;

        if (!radio.available(&pipe)) {
            continue;
        }

        radio.read(buffer, sizeof(buffer));
        if (buffer[0] == static_cast<uint8_t>(PacketTag::Log)) {
            /* The car's own lines explain a refusal, show them live */
            drainLogFragment(pipe, buffer);
            continue;
        }
        if (buffer[0] != static_cast<uint8_t>(PacketTag::Ota)) {
            continue; /* a packet meant for another reader */
        }

        memcpy(&packet, buffer, sizeof(packet));
        if (packet.kind == static_cast<uint8_t>(OtaKind::Error)) {
            errorCode = packet.data[0];
            return OtaWait::Error;
        }
        if (packet.kind == static_cast<uint8_t>(OtaKind::Ack) && packet.seq == seq) {
            return OtaWait::Ok;
        }
    }

    return OtaWait::Timeout;
}

void runOtaStream()
{
    uint8_t header[8];
    uint8_t chunk[sizeof(OtaPacket::data)];
    uint32_t length = 0;
    uint32_t crc = 0;
    uint16_t seq = 0;
    uint8_t error = 0;

    /* A device command leaves on the car pipe, whatever the current mode */
    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    if (!readExact(header, sizeof(header), kOtaTimeoutMs)) {
        Serial.println(F("[ota] no header, dropped"));
        return;
    }

    length = static_cast<uint32_t>(header[0]) | (static_cast<uint32_t>(header[1]) << 8) |
             (static_cast<uint32_t>(header[2]) << 16) | (static_cast<uint32_t>(header[3]) << 24);
    crc = static_cast<uint32_t>(header[4]) | (static_cast<uint32_t>(header[5]) << 8) |
          (static_cast<uint32_t>(header[6]) << 16) | (static_cast<uint32_t>(header[7]) << 24);

    Serial.print(F("[ota] image "));
    Serial.print(length);
    Serial.print(F(" bytes, crc 0x"));
    Serial.println(crc, HEX);

    /* The radio acknowledgement is not the signal that matters here: the
     * car erases the staging sector for two seconds and its FIFO refuses
     * packets meanwhile. What counts is the application's answer, after it */
    bool accepted = false;
    for (uint8_t attempt = 0; attempt < 3 && !accepted; ++attempt) {
        sendOtaBegin(length, crc);

        const OtaWait begin = waitOtaResponse(kOtaBeginAck, 5000, error);
        if (begin == OtaWait::Ok) {
            accepted = true;
        } else if (begin == OtaWait::Error) {
            Serial.print(F("[ota] begin refused, error "));
            Serial.println(error);
            drainSerial(300);
            return;
        }
    }

    if (!accepted) {
        Serial.println(F("[ota] begin not accepted"));
        drainSerial(300);
        return;
    }
    Serial.println(F("[ota] begin accepted"));

    while (length > 0) {
        const uint8_t take = length > sizeof(chunk) ? sizeof(chunk) : static_cast<uint8_t>(length);
        bool delivered = false;

        if (!readExact(chunk, take, kOtaTimeoutMs)) {
            Serial.println(F("[ota] stream stopped, image incomplete"));
            drainSerial(300);
            return;
        }

        /* The tail of the last packet is padded with the erased-flash value;
         * the receiver only counts the bytes the record covers */
        memset(&chunk[take], 0xFF, sizeof(chunk) - take);

        for (uint8_t attempt = 0; attempt < 3 && !delivered; ++attempt) {
            sendOtaPacket(OtaKind::Data, seq, chunk, sizeof(chunk));

            const OtaWait wait = waitOtaResponse(seq, 500, error);
            if (wait == OtaWait::Ok) {
                delivered = true;
            } else if (wait == OtaWait::Error) {
                Serial.print(F("[ota] receiver error "));
                Serial.println(error);
                Serial.write(kOtaNak);
                drainSerial(300);
                return;
            }
        }

        if (!delivered) {
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }

        Serial.write(kOtaAck);
        ++seq;
        length -= take;
    }

    bool ended = false;
    for (uint8_t attempt = 0; attempt < 3 && !ended; ++attempt) {
        sendOtaPacket(OtaKind::End, seq, nullptr, 0);

        const OtaWait wait = waitOtaResponse(seq, 5000, error);
        if (wait == OtaWait::Ok) {
            ended = true;
        } else if (wait == OtaWait::Error) {
            Serial.print(F("[ota] receiver error "));
            Serial.println(error);
            return;
        }
    }

    Serial.println(ended ? F("[ota] image accepted") : F("[ota] end not accepted"));
    /* One last status byte: the last chunk was acknowledged before the car
     * verified the CRC, so without this the PC could not report the outcome */
    Serial.write(ended ? kOtaAck : kOtaNak);
}

void runOtaSelfTest()
{
    uint8_t chunk[sizeof(OtaPacket::data)];

    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    for (uint8_t index = 0; index < sizeof(chunk); ++index) {
        chunk[index] = index;
    }

    Serial.println(F("[ota] test image: Begin, two Data, End"));
    Serial.print(F("[ota] begin "));
    Serial.println(sendOtaBegin(2 * sizeof(chunk), 0x12345678UL) ? F("ACK") : F("no ACK"));

    for (uint16_t seq = 0; seq < 2; ++seq) {
        chunk[0] = static_cast<uint8_t>(seq);
        Serial.print(F("[ota] data "));
        Serial.print(seq);
        Serial.print(' ');
        Serial.println(sendOtaPacket(OtaKind::Data, seq, chunk, sizeof(chunk)) ? F("ACK") :
                                                                                 F("no ACK"));
    }

    Serial.print(F("[ota] end "));
    Serial.println(sendOtaPacket(OtaKind::End, 2, nullptr, 0) ? F("ACK") : F("no ACK"));
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
    case 'f':
        /* A device command, not a motor one: it works from any mode and
         * switching to the box mode is part of sending it */
        sendDfuRequest();
        break;
    case 'o':
        runOtaSelfTest();
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
        const int value = Serial.read();

        if (value == kOtaStart) {
            runOtaStream();
        } else {
            handleKey(static_cast<char>(value));
        }
    }

    /* Short, not 100 ms: the receiver's FIFO is three packets deep, so a burst
     * of car logs during boot overflows it and whole lines are lost. Draining
     * often is what keeps a log readable */
    delay(5);
}

void printHelp()
{
    Serial.println("\n=== NrfLink - Gaspetto radio link tool ===");
    Serial.println("Modes:");
    Serial.println("  1: car emulation (default) - reads the program the Box sends");
    Serial.println("     on _car_ and answers with telemetry and logs on _box_");
    Serial.println("  2: box/sender - sends commands on _car_, reads telemetry on _box_");
    Serial.println("  ?: this help");
    Serial.println("  f: ask the car to enter its DFU bootloader (works in any mode)");
    Serial.println("  o: send a dummy OTA image (framing check)");
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

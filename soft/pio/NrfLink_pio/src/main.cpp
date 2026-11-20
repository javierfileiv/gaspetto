#include "Arduino.h"
#include "CommandPacket.h"
#include "Crc32.h"
#include "Events.h"
#include "Log.h"
#include "OtaProtocol.h"
#include "RF24.h"
#include "RadioProtocol.h"
#include "config_radio.h"
#include "radio_send.h"

#include <Arduino.h>
#include <avr/wdt.h>
#include <string.h>

/* Mode 1 emulates the car: it listens for the program the Box sends on
 * _car_ and answers with telemetry on _box_, where the box listens. Mode 2
 * emulates the box: it sends commands on _car_ and listens for telemetry
 * on _box_. Mode 3 is quiet: the chip leaves RX, nothing is transmitted and
 * the log lines stay on the console, so the tool can sit between two real
 * devices without disturbing their link. The car is the default because the
 * bench test drives the box side. */
constexpr uint8_t kModeCar = 1;
constexpr uint8_t kModeBox = 2;
constexpr uint8_t kModeQuiet = 3;

/* Telemetry every 500 ms while a simulated program runs, like the car: it
 * sends none when it is idle, so the stream follows the execution instead of
 * running for its own sake */
constexpr uint32_t kTelemetryPeriodMs = 500;

/* The chip retransmits twice before it gives up, so this bounds the wait
 * after MAX_RT rather than the whole exchange; the verdict still comes from
 * txStandBy, unlike radio.write() which returns before the ACK */
constexpr uint32_t kTxTimeoutMs = 10;

/* A simulated command takes this long, which is about the car's quarter turn:
 * the box then has a real program window to animate */
constexpr uint32_t kMovementDurationMs = 3000;
constexpr uint8_t kProgramCapacity = BOX_MAX_PAYLOAD_COMMANDS;

RF24 radio(NRF24_CE, NRF24_CSN);

uint8_t gMode = kModeCar;

/* Car emulation plays a received program command by command and sends the
 * same end notice the real car sends: without it the box's yellow breath
 * never stops, and the bench cannot replace the car */
uint8_t gProgramCommands[kProgramCapacity];
uint8_t gProgramCount = 0;
uint8_t gProgramIndex = 0;
uint32_t gProgramStartMs = 0;
bool gProgramActive = false;

/* Car-mode faults: each one removes a single thing the real car does, so the
 * box can be watched reacting to the failure without unplugging anything */
bool gNoProgramDone = false;
bool gNoPingAnswer = false;
bool gNoAck = false;
bool gNoChatter = false;

/* Mode to give back after the post-OTA log window, 0 when none is pending */
uint8_t gPendingMode = 0;
uint32_t gPendingModeAt = 0;
uint32_t gLastTelemetryMs = 0;
uint16_t gTelemetryTick = 0;

/* One simulated car log line every kLogPeriodMs while a program runs, offset
 * from the telemetry period so the fragments and the telemetry never land on
 * the same beat. At rest the emulator stays silent, like the car, so a box
 * just flashed shows nothing until a program is sent */
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
void sendIrqTest();
void drainRadio();
void runOtaStream();
void runOtaSelfTest();
void drainLogFragment(uint8_t pipe, const uint8_t *payload);
void displayTelemetry(const TelemetryPacket &t);
void displayCommandPacket(const CommandPacket &pkt);
void displayVersion(const VersionPacket &version);
void sendVersionRequest();
void respondVersion();
void queueProgram(const CommandPacket &packet);
void serviceCarProgram();
void sendProgramDone();

void setup()
{
    /* A watchdog reset leaves the WDT armed with its 15 ms timeout, so the
     * board resets again before setup() can do anything: the reset word would
     * turn into a reset loop. Clear the flag and stop the timer first */
    MCUSR = 0;
    wdt_disable();

    Serial.begin(115200);
#ifdef ARDUINO
    while (!Serial) {
        delay(10);
    }
#endif
    Serial.println(F("NrfLink - Gaspetto radio link tool"));
    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {
        } /* Hold in infinite loop. */
    }
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    /* Three retransmissions instead of the fifteen the library defaults to:
     * an unanswered write must not block here for twenty milliseconds, or the
     * image the PC keeps sending overflows the serial buffer and the whole
     * stream shifts by the bytes we dropped */
    radio.setRetries(2, 3);
    radio.setAddressWidth(5);
    /* Both payloads are 32 bytes, CommandPacket sets the size for both */
    radio.setPayloadSize(sizeof(CommandPacket));
    radio.powerUp();
    /* The chip answered begin(): log fragments can now ride the air instead
     * of the console */
    Log::markNrf24Ready();

    setMode(kModeCar);

    /* printDetails() and printPrettyDetails() format through printf, which
     * needs printf_begin() and a stdout binding this tool never sets: their
     * output went nowhere and the call was the last thing before the reset.
     * setMode() already prints the addresses and the [mode] line */
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
    gPendingMode = 0;

    /* The program queue belongs to car emulation: leaving the mode drops it,
     * or a stale end notice would arrive after the operator moved on */
    if (gMode != kModeCar) {
        gProgramActive = false;
        gProgramCount = 0;
        gProgramIndex = 0;

        /* The ACK injection would break the OTA relay, whose pipe setup this
         * mode switch is usually preparing */
        if (gNoAck) {
            gNoAck = false;
            radio.setAutoAck(true);
        }
    }

    if (gMode == kModeQuiet) {
        /* Leaving RX mode stops the chip from hearing a packet and from
         * answering its acknowledge; the sends are guarded in drainRadio()
         * and the loop, and the log transport falls back to the console so
         * the operator keeps the lines without a byte on the air */
        Log::setMuted(true);
        radio.stopListening();
        Serial.println(F("[mode] 3 = quiet, radio idle, console only"));
        return;
    }

    Log::setMuted(false);

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
    Serial.print(F("[mode] "));
    Serial.print(gMode == kModeCar ? "1 = car emulation" : "2 = box/sender");
    Serial.print(F(", tx "));
    printAddress(gMode == kModeCar ? gbox_pipe_name : gcar_pipe_name);
    Serial.print(F(", rx(pipe1) "));
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

    sent = radio_send(radio, &t, sizeof(t), kTxTimeoutMs);

    Serial.print(F("[tx] telemetry #"));
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

/* How long a command takes in the simulation: movements last, stops and queue
 * clears take effect at once, like the car */
uint32_t commandDurationMs(uint8_t command)
{
    switch (static_cast<CommandId>(command)) {
    case CommandId::MOTOR_FORWARD:
    case CommandId::MOTOR_BACKWARD:
    case CommandId::MOTOR_TURN_LEFT:
    case CommandId::MOTOR_TURN_RIGHT:
        return kMovementDurationMs;
    default:
        return 0;
    }
}

void startNextCommand()
{
    Serial.print(F("[car] running "));
    Serial.print(commandIdToString(static_cast<CommandId>(gProgramCommands[gProgramIndex])));
    Serial.print(F(" for "));
    Serial.print(commandDurationMs(gProgramCommands[gProgramIndex]));
    Serial.println(F(" ms"));
}

void queueProgram(const CommandPacket &packet)
{
    uint8_t accepted = 0;

    for (uint8_t i = 0; i < packet.count && gProgramCount < kProgramCapacity; ++i) {
        gProgramCommands[gProgramCount++] = packet.commands[i];
        ++accepted;
    }

    Serial.print(F("[car] program queued: "));
    Serial.print(static_cast<int>(accepted));
    Serial.print(F(" command(s), "));
    Serial.print(static_cast<int>(gProgramCount));
    Serial.println(F(" pending"));

    if (!gProgramActive && gProgramCount > 0) {
        gProgramActive = true;
        gProgramIndex = 0;
        gProgramStartMs = millis();
        startNextCommand();
    }
}

void serviceCarProgram()
{
    if (!gProgramActive) {
        return;
    }

    if ((millis() - gProgramStartMs) < commandDurationMs(gProgramCommands[gProgramIndex])) {
        return;
    }

    Serial.print(F("[car] finished "));
    Serial.println(commandIdToString(static_cast<CommandId>(gProgramCommands[gProgramIndex])));

    ++gProgramIndex;
    gProgramStartMs = millis();

    /* Instant commands are consumed in the same pass: waiting a whole loop for
     * a stop would make the batch feel slower than the car */
    while (gProgramIndex < gProgramCount &&
           commandDurationMs(gProgramCommands[gProgramIndex]) == 0) {
        Serial.print(F("[car] finished "));
        Serial.println(commandIdToString(static_cast<CommandId>(gProgramCommands[gProgramIndex])));
        ++gProgramIndex;
    }

    if (gProgramIndex < gProgramCount) {
        startNextCommand();
        return;
    }

    gProgramActive = false;
    gProgramCount = 0;
    gProgramIndex = 0;
    sendProgramDone();
}

/* Same notice and same retries as the car: the box must not miss it, or its
 * breath never stops */
void sendProgramDone()
{
    constexpr uint8_t kAttempts = 10;

    if (gNoProgramDone) {
        Serial.println(F("[fault] ProgramDone suppressed"));
        return;
    }
    constexpr uint32_t kRetryGapMs = 40;
    uint8_t payload[sizeof(CommandPacket)] = {};

    payload[0] = static_cast<uint8_t>(PacketTag::ProgramDone);

    for (uint8_t attempt = 0; attempt < kAttempts; ++attempt) {
        if (radio_send(radio, payload, sizeof(payload), kTxTimeoutMs)) {
            Serial.print(F("[program] done ACKed on try "));
            Serial.println(attempt + 1);
            return;
        }
        delay(kRetryGapMs);
    }

    Serial.println(F("[program] done never acknowledged"));
}

/* The box pings the car with a VersionRequest: emulation has to answer, or the
 * box would call the link lost with no car around */
void respondVersion()
{
    VersionPacket answer = {};

    if (gNoPingAnswer) {
        Serial.println(F("[fault] ping left unanswered"));
        return;
    }

    answer.tag = static_cast<uint8_t>(PacketTag::Version);
    answer.wire = kWireProtocolVersion;
    answer.build = 0; /* this tool carries no commit stamp of its own */
    strncpy(answer.version, "nrf-link", sizeof(answer.version) - 1);

    radio_send(radio, &answer, sizeof(answer), kTxTimeoutMs);
}

void displayCommandPacket(const CommandPacket &pkt)
{
    uint8_t i;

    Serial.print(F("[rx] CommandPacket count="));
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

    sent = radio_send(radio, &pkt, sizeof(pkt), kTxTimeoutMs);

    Serial.print(sent ? "Sent command: " : "FAILED to send: ");
    Serial.println(commandIdToString(cmd));
}

/* The version request doubles as a ping: the answer proves the other
 * application is running, and the delay is the round trip */
uint32_t ping_sent_ms = 0;
bool ping_pending = false;

void sendVersionRequest()
{
    VersionPacket request = {};

    /* Car emulation transmits on _box_, so the request would never reach the
     * car: the box mode owns the _car_ writing pipe */
    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    request.tag = static_cast<uint8_t>(PacketTag::VersionRequest);

    const bool sent = radio_send(radio, &request, sizeof(request), kTxTimeoutMs);

    if (sent) {
        ping_sent_ms = millis();
        ping_pending = true;
    }

    Serial.println(sent ? F("[version] ping sent, waiting for the answer") :
                          F("[version] ping FAILED (no answer from the chip)"));
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

    sent = radio_send(radio, payload, sizeof(payload), kTxTimeoutMs);

    Serial.print(sent ? F("DFU request ACKed by the car") : F("DFU request FAILED (no ACK)"));
    Serial.println(F(", the board should re-enumerate in about a second"));
}

/* Ask the car what its IRQ pin saw. The request is a real reception, so a
 * wired pin has already counted at least one falling edge when the car
 * answers. A car that ACKs but reports zero has the pin unconnected, which
 * the radio acknowledgement alone cannot tell: the chip answered either way */
void sendIrqTest()
{
    uint8_t payload[sizeof(CommandPacket)] = { 0 };
    bool sent;

    payload[0] = static_cast<uint8_t>(PacketTag::Event);
    payload[1] = static_cast<uint8_t>(EventId::IRQ_TEST);
    payload[2] = static_cast<uint8_t>(CommandId::NONE);

    /* Same reason as the DFU request: the _car_ writing pipe belongs to the
     * box mode, and the car logs come back on _box_ which that mode reads */
    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    sent = radio_send(radio, payload, sizeof(payload), kTxTimeoutMs);

    Serial.print(sent ? F("IRQ test ACKed by the car") : F("IRQ test FAILED (no ACK)"));
    Serial.println(F(", its A4 report follows"));
}

void addCommandToProgram(CommandId cmd)
{
    if (currentProgram.count < BOX_MAX_PAYLOAD_COMMANDS) {
        currentProgram.commands[currentProgram.count++] = static_cast<uint8_t>(cmd);
        Serial.print(F("Added to program: "));
        Serial.print(commandIdToString(cmd));
        Serial.print(F(" (count: "));
        Serial.print(currentProgram.count);
        Serial.println(F(")"));
    } else {
        Serial.println(F("Program is full!"));
    }
}

void sendProgram()
{
    bool sent;

    if (currentProgram.count == 0) {
        Serial.println(F("Program is empty!"));
        return;
    }

    sent = radio_send(radio, &currentProgram, sizeof(currentProgram), kTxTimeoutMs);

    Serial.print(sent ? "Sent program with " : "FAILED to send program with ");
    Serial.print(currentProgram.count);
    Serial.println(F(" commands"));

    currentProgram = {}; /* Reset program. */
}

void clearProgram()
{
    currentProgram = {};
    Serial.println(F("Program cleared"));
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
            Serial.println(F("Resetting the board..."));
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

    if (gMode == kModeQuiet) {
        return;
    }

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
        } else if (payload[0] == static_cast<uint8_t>(PacketTag::Version)) {
            VersionPacket version;

            memcpy(&version, payload, sizeof(version));
            displayVersion(version);
        } else if (payload[0] == static_cast<uint8_t>(PacketTag::VersionRequest)) {
            respondVersion();
        } else if (payload[0] == static_cast<uint8_t>(PacketTag::ProgramDone)) {
            Serial.println(F("[program] done: the car finished the batch"));
        } else if (gMode == kModeCar) {
            CommandPacket cmd;

            memcpy(&cmd, payload, sizeof(cmd));
            displayCommandPacket(cmd);

            /* With the acknowledge off the box sees its send fail: the car
             * must look unreachable, not run the program four times as the
             * sender retransmits */
            if (gNoAck) {
                Serial.println(F("[fault] program dropped (ACK OFF)"));
            } else {
                queueProgram(cmd);
            }
        } else if (payload[0] == static_cast<uint8_t>(PacketTag::Telemetry)) {
            TelemetryPacket tlm;

            memcpy(&tlm, payload, sizeof(tlm));
            displayTelemetry(tlm);
        } else {
            Serial.print(F("[rx] ignored packet, tag=0x"));
            Serial.println(payload[0], HEX);
        }
    }
}

/* ========================================== */
/* OTA relay                                  */
/* ========================================== */
/* The PC streams a binary image on this port: a 0x01 start byte, the image
 * length and its CRC as two little-endian words, then the image itself. Every
 * 28-byte chunk becomes one Data packet. The nRF24 acknowledges each packet at
 * the hardware level, so the car never answers during the data phase: it only
 * speaks once after the erase, once for the CRC, and on a real error. That is
 * what keeps the channel out of the turnaround race two thousand times over */
constexpr uint8_t kOtaStart = 0x01;
/* Control codes, not letters: NrfLink prints hex addresses and log lines
 * on the same port, and a plain 'F' would be read as a refusal */
constexpr uint8_t kOtaAck = 0x06;
constexpr uint8_t kOtaNak = 0x15;
/* Flow control, and the only byte that travels during the data phase: one
 * credit per group of packets means "I have taken that much, send more" */
constexpr uint8_t kOtaCredit = 0x11;
constexpr uint8_t kOtaCreditPackets = 4;
constexpr uint16_t kOtaTimeoutMs = 5000;
/* The car erases the staging sector before it can take a byte, and writes the
 * record through another erase when the image is complete */
constexpr uint16_t kOtaBeginAckMs = 6000;
constexpr uint16_t kOtaEndAckMs = 10000;
/* The PC keeps the port open after the verdict to read the car's boot lines,
 * and those ride _box_: stay in the box mode for the same window */
constexpr uint32_t kPostOtaModeDelayMs = 15000;
/* A failed write already means the car's FIFO was full and cost two
 * milliseconds of hardware retransmissions. Three tries cover a short backlog
 * and still keep the whole stall under ten milliseconds, which is what the
 * serial buffer can absorb while the PC keeps sending */
constexpr uint8_t kOtaWriteAttempts = 3;

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

    return radio_send(radio, &packet, sizeof(packet), kTxTimeoutMs);
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

/* Wait for one of the car's rare answers. The radio acknowledgement only says
 * a packet reached the chip; this one carries what the car decided */
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

/* The car only speaks up to complain, so a look between two packets is enough:
 * waiting here would give the channel away for nothing */
bool pollReceiverError(uint8_t &errorCode)
{
    uint8_t pipe;

    while (radio.available(&pipe)) {
        uint8_t buffer[sizeof(OtaPacket)];
        OtaPacket packet;

        radio.read(buffer, sizeof(buffer));
        if (buffer[0] == static_cast<uint8_t>(PacketTag::Log)) {
            drainLogFragment(pipe, buffer);
            continue;
        }
        if (buffer[0] != static_cast<uint8_t>(PacketTag::Ota)) {
            continue;
        }

        memcpy(&packet, buffer, sizeof(packet));
        if (packet.kind == static_cast<uint8_t>(OtaKind::Error)) {
            errorCode = packet.data[0];
            return true;
        }
    }

    return false;
}

/* The OTA entry borrows the box mode to reach the car. This puts the tool back
 * where the operator left it on every exit path, so a quiet tool stays quiet
 * and a car emulator resumes its telemetry once the transfer is over */
class OtaModeGuard {
public:
    OtaModeGuard(uint8_t entryMode, bool &carRestarts)
            : _entryMode(entryMode)
            , _carRestarts(carRestarts)
    {
    }

    ~OtaModeGuard()
    {
        if (gMode == _entryMode) {
            return;
        }

        /* An accepted image makes the car reset, and its boot lines ride _box_:
         * stay in the box mode for the window the PC keeps the port open, then
         * give the operator's mode back from the loop */
        if (_carRestarts) {
            gPendingMode = _entryMode;
            gPendingModeAt = millis();
            return;
        }

        setMode(_entryMode);
    }

private:
    uint8_t _entryMode;
    bool &_carRestarts;
};

void runOtaStream()
{
    uint8_t header[8];
    uint8_t chunk[sizeof(OtaPacket::data)];
    uint32_t length = 0;
    uint32_t crc = 0;
    uint32_t expected = 0;
    uint32_t received = 0;
    uint32_t streamCrc = kCrc32Init;
    uint16_t seq = 0;
    uint16_t radioRetries = 0;
    uint8_t error = 0;
    bool carRestarts = false;
    const OtaModeGuard restoreMode(gMode, carRestarts);

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

    expected = length;

    Serial.print(F("[ota] image "));
    Serial.print(length);
    Serial.print(F(" bytes, crc 0x"));
    Serial.println(crc, HEX);

    /* The radio acknowledgement is not the signal that matters here: the car
     * erases the staging sector for two seconds and its FIFO refuses packets
     * meanwhile. What counts is the application's answer, after it */
    bool accepted = false;
    for (uint8_t attempt = 0; attempt < 3 && !accepted; ++attempt) {
        Serial.print(F("[ota] begin attempt "));
        Serial.print(attempt + 1);
        Serial.println(sendOtaBegin(length, crc) ? F(", the radio acknowledged it") :
                                                   F(", no answer from the chip"));

        const OtaWait begin = waitOtaResponse(kOtaBeginAck, kOtaBeginAckMs, error);
        if (begin == OtaWait::Ok) {
            accepted = true;
        } else if (begin == OtaWait::Error) {
            Serial.print(F("[ota] begin refused, error "));
            Serial.println(error);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }
    }

    if (!accepted) {
        Serial.println(F("[ota] begin not accepted"));
        Serial.write(kOtaNak);
        drainSerial(300);
        return;
    }

    Serial.println(F("[ota] begin accepted"));
    /* The image waits until the car is listening again: the PC takes this byte
     * as the order to start, and would otherwise overflow our serial buffer
     * while we are still busy with the answer above */
    Serial.write(kOtaAck);

    while (length > 0) {
        const uint8_t take = length > sizeof(chunk) ? sizeof(chunk) : static_cast<uint8_t>(length);
        bool delivered = false;

        if (!readExact(chunk, take, kOtaTimeoutMs)) {
            /* The PC cannot be blamed any more: it never sends more than the
             * credits below allow, so a shortfall here means the serial link
             * itself lost bytes */
            Serial.print(F("[ota] stream stopped after "));
            Serial.print(received);
            Serial.print(F(" of "));
            Serial.print(expected);
            Serial.print(F(" bytes, radio retries "));
            Serial.println(radioRetries);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }

        /* The tail of the last packet is padded with the erased-flash value;
         * the receiver only counts the bytes the record covers */
        memset(&chunk[take], 0xFF, sizeof(chunk) - take);

        for (uint8_t attempt = 0; attempt < kOtaWriteAttempts && !delivered; ++attempt) {
            delivered = sendOtaPacket(OtaKind::Data, seq, chunk, sizeof(chunk));
            if (!delivered) {
                ++radioRetries;
                /* The car reads one packet per loop, so a full FIFO needs a
                 * few milliseconds to free a slot: burning the three attempts
                 * at once would report a refusal it could have avoided */
                delay(3);
            }
        }

        if (!delivered) {
            Serial.print(F("[ota] radio refused packet "));
            Serial.println(seq);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }

        if (pollReceiverError(error)) {
            Serial.print(F("[ota] receiver error "));
            Serial.println(error);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }

        received += take;
        /* Only the bytes the image really covers: the tail of the last packet
         * is padding, and the PC's CRC does not include it */
        streamCrc = crc32Update(streamCrc, chunk, take);
        ++seq;
        length -= take;

        /* The car reads one packet per loop, and its FIFO only holds three:
         * sending the next 28 bytes immediately would fill it and the packet
         * after that would be refused. One millisecond keeps the pipe full
         * without overrunning it */
        delay(1);

        /* One credit per full group of packets. The PC may not send more than
         * its credits allow, which keeps this board's serial buffer from
         * overflowing: without it the PC runs ahead, the buffer fills, and the
         * UART drops the surplus without anyone noticing */
        if (seq % kOtaCreditPackets == 0) {
            Serial.write(kOtaCredit);
        }
    }

    /* The PC is the authority on the image, so the relay only has to check
     * that what it received is what the PC announced. Catching it here says
     * the loss happened on the serial link, not in the car */
    streamCrc = crc32Finish(streamCrc);
    if (streamCrc != crc) {
        Serial.print(F("[ota] stream corrupt: crc 0x"));
        Serial.print(streamCrc, HEX);
        Serial.print(F(" instead of 0x"));
        Serial.println(crc, HEX);
        Serial.write(kOtaNak);
        drainSerial(300);
        return;
    }

    bool ended = false;
    for (uint8_t attempt = 0; attempt < 3 && !ended; ++attempt) {
        sendOtaPacket(OtaKind::End, seq, nullptr, 0);

        const OtaWait wait = waitOtaResponse(seq, kOtaEndAckMs, error);
        if (wait == OtaWait::Ok) {
            ended = true;
        } else if (wait == OtaWait::Error) {
            Serial.print(F("[ota] receiver error "));
            Serial.println(error);
            break;
        }
    }

    carRestarts = ended;
    Serial.println(ended ? F("[ota] image accepted") : F("[ota] end not accepted"));
    Serial.write(ended ? kOtaAck : kOtaNak);
}
void runOtaSelfTest()
{
    uint8_t chunk[sizeof(OtaPacket::data)];
    bool carRestarts = false;
    const OtaModeGuard restoreMode(gMode, carRestarts);

    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    for (uint8_t index = 0; index < sizeof(chunk); ++index) {
        chunk[index] = index;
    }

    Serial.println(F("[ota] test image: Begin, two Data, End"));
    Serial.print(F("[ota] begin "));
    const bool beginSent = sendOtaBegin(2 * sizeof(chunk), 0x12345678UL);
    Serial.println(beginSent ? F("ACK") : F("no ACK"));

    if (!beginSent) {
        return;
    }

    /* The Begin makes the car erase its staging sector for about two seconds.
     * Its RF FIFO only holds three packets and nobody drains it while the app
     * is busy, so sending the rest now would have the End dropped without an
     * acknowledgement. Wait for the application answer first, like the real
     * relay does */
    uint8_t error = 0;
    const OtaWait begin = waitOtaResponse(kOtaBeginAck, kOtaBeginAckMs, error);
    Serial.println(begin == OtaWait::Ok ? F("[ota] begin accepted") :
                                          F("[ota] begin refused or silent"));

    if (begin != OtaWait::Ok) {
        return;
    }

    for (uint16_t seq = 0; seq < 2; ++seq) {
        chunk[0] = static_cast<uint8_t>(seq);
        Serial.print(F("[ota] data "));
        Serial.print(seq);
        Serial.print(' ');
        Serial.println(sendOtaPacket(OtaKind::Data, seq, chunk, sizeof(chunk)) ? F("ACK") :
                                                                                 F("no ACK"));
        /* Leave the car time to read the packet and program its 28 bytes */
        delay(10);
    }

    Serial.print(F("[ota] end "));
    Serial.println(sendOtaPacket(OtaKind::End, 2, nullptr, 0) ? F("ACK") : F("no ACK"));
}

void handleKey(char ch)
{
    if (consumeResetWord(ch)) {
        return;
    }

    /* A key means the operator is driving again: the mode they pick, or the
     * one they leave in place, wins over a pending post-OTA restore */
    gPendingMode = 0;

    switch (ch) {
    case '1':
        setMode(kModeCar);
        break;
    case '2':
        setMode(kModeBox);
        break;
    case '3':
        setMode(kModeQuiet);
        break;
    case 'f':
        /* A device command, not a motor one: it works from any mode and
         * switching to the box mode is part of sending it */
        sendDfuRequest();
        break;
    case 'i':
        sendIrqTest();
        break;
    case 'o':
        runOtaSelfTest();
        break;
    case 'v':
        sendVersionRequest();
        break;
    case 'p':
        if (gMode != kModeCar) {
            break;
        }
        gNoProgramDone = !gNoProgramDone;
        Serial.println(gNoProgramDone ? F("[fault] ProgramDone OFF") : F("[fault] ProgramDone ON"));
        break;
    case 'l':
        if (gMode != kModeCar) {
            break;
        }
        gNoPingAnswer = !gNoPingAnswer;
        Serial.println(gNoPingAnswer ?
                               F("[fault] ping answers OFF, the box should call the link lost") :
                               F("[fault] ping answers ON"));
        break;
    case 'n':
        if (gMode != kModeCar) {
            break;
        }
        gNoAck = !gNoAck;
        radio.setAutoAck(!gNoAck);
        Serial.println(gNoAck ? F("[fault] ACK OFF, the box's sends will fail") :
                                F("[fault] ACK ON"));
        break;
    case 't':
        if (gMode != kModeCar) {
            break;
        }
        gNoChatter = !gNoChatter;
        Serial.println(gNoChatter ? F("[fault] telemetry and logs OFF") :
                                    F("[fault] telemetry and logs ON"));
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
    /* Queued log fragments: one per pass, after the received traffic */
    Log::service();

    /* The post-OTA window is over: give the operator's mode back */
    if (gPendingMode != 0 && (millis() - gPendingModeAt) >= kPostOtaModeDelayMs) {
        const uint8_t mode = gPendingMode;

        gPendingMode = 0;
        Serial.print(F("[ota] log window over, back to mode "));
        Serial.println(mode);
        setMode(mode);
    }

    if (gMode == kModeCar) {
        serviceCarProgram();
    }

    if (gMode == kModeCar && gProgramActive && !gNoChatter &&
        (millis() - gLastTelemetryMs) >= kTelemetryPeriodMs) {
        gLastTelemetryMs = millis();
        sendTelemetry();
    }

    if (gMode == kModeCar && gProgramActive && !gNoChatter &&
        (millis() - gLastLogMs) >= kLogPeriodMs) {
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
    Serial.println(F("\n=== NrfLink - Gaspetto radio link tool ==="));
    Serial.println(F("Modes:"));
    Serial.println(F("  1: car emulation (default) - plays the program the Box sends"));
    Serial.println(F("     on _car_, answers its ping, then reports the end on _box_"));
    Serial.println(F("  2: box/sender - sends commands on _car_, reads telemetry on _box_"));
    Serial.println(F("  3: quiet - radio idle, nothing sent or received, console only"));
    Serial.println(F("  ?: this help"));
    Serial.println(F("  f: ask the car to enter its DFU bootloader (works in any mode)"));
    Serial.println(F("  i: ask the car what its IRQ pin saw on A4 (works in any mode)"));
    Serial.println(F("  v: ping the other device: its build and the round-trip time"));
    Serial.println(F("\nCar mode faults (mode 1 only):"));
    Serial.println(F("  p: stop sending ProgramDone"));
    Serial.println(F("  l: stop answering the box ping"));
    Serial.println(F("  n: stop acknowledging the box sends"));
    Serial.println(F("  t: stop telemetry and logs"));
    Serial.println(F("  o: send a dummy OTA image (framing check)"));
    Serial.println(F("  reset: type the five letters in a row to reboot the MCU"));
    Serial.println(F("\nMode 2 keys:"));
    Serial.println(F("  w/s/a/d/x: send a single command"));
    Serial.println(F("  W/S/A/D/X: add to the program queue"));
    Serial.println(F("  *: send the queue, -: clear the queue"));
    Serial.println(F("========================================\n"));
}

/* The packet carries an absolute target and the raw yaw, so the difference
 * alone can read -270 where the car is really turning +90 the other way */
float wrapAngle(float degrees)
{
    while (degrees > 180.0f) {
        degrees -= 360.0f;
    }
    while (degrees < -180.0f) {
        degrees += 360.0f;
    }
    return degrees;
}

void displayVersion(const VersionPacket &version)
{
    Serial.print(F("[version] build "));
    Serial.print(static_cast<int>(version.build));
    Serial.print(F(" ("));
    Serial.print(version.version);
    Serial.print(F("), wire protocol v"));
    Serial.print(static_cast<int>(version.wire));
    if (ping_pending) {
        Serial.print(F(", pong in "));
        Serial.print(millis() - ping_sent_ms);
        Serial.print(F(" ms"));
        ping_pending = false;
    }
    Serial.println();
}

void displayTelemetry(const TelemetryPacket &t)
{
    Serial.print(F("TLM yaw="));
    Serial.print(t.yaw, 1);
    Serial.print(F(" tgt="));
    Serial.print(t.targetYaw, 1);
    Serial.print(F(" err="));
    Serial.print(wrapAngle(t.targetYaw - t.yaw), 2); /* shortest way, as the car sees it */
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

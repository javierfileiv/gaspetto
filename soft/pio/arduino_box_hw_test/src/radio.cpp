/* Radio side of the bench: the program send test, the radio menu and the
 * car stream listener */
#include "CommandPacket.h"
#include "PieceProgram.h"
#include "RadioProtocol.h"
#include "hwtest.h"
#include "pin_definitions.h"

#include <Arduino.h>
#include <config_radio.h>
#include <string.h>

/* ========================================== */
/* Send the scanned board over the radio, key "p" */
/* ========================================== */

/* The LED verdict only shows if the 5V rail is up; calibration keeps it
 * down, so raise it for the animation and put it back, the same contract
 * the LED menu uses */
static void runSendFeedback(void (*animation)())
{
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);
    delay(kRailSettleDelayMs);
    animation();
    blackoutLeds();
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
}

/* Scan -> build -> transmit, the chain production GBox runs when it
 * validates a program, without GBox itself: the decode is the compiled
 * per-slot row (pieceForRaw, the one walk), the build is the shared
 * buildProgramFromPieces, only the transport is this bench. The verdict
 * mirrors the production LED semantics: green on ACK, amber on an empty
 * board, red on a refused build, RF error when the car does not answer */
void runSendPiecesTest()
{
    BoxBoardPieces board = {};
    CommandPacket packet = {};
    bool isEmpty         = true;
    bool hasLoopCall     = false;
    bool built           = false;

    Serial.println("\n--- SEND PIECES: scan -> build -> radio ---");

    if (!radio_ok)
    {
        Serial.println("[send] radio is not up: re-probe with 's' in health mode");
        return;
    }
    if (test_mode != TestMode::Calibration)
    {
        Serial.println("[send] calibration mode required ('c' to switch)");
        return;
    }

    /* Main zone first, in slot order: an EMPTY ends the sequence and leaves
     * the slots after it alone - they stay EMPTY from the initialisation */
    if (!scanZone(0, BOX_MAIN_SLOTS, board, hasLoopCall))
    {
        Serial.println("[send] INVALID slot: aborting, nothing sent");
        runSendFeedback(runBuildErrorAnimationTest);
        return;
    }

    /* The loop zone is read only when the main zone held a LOOP_CALL:
     * without one those slots never reach the program */
    if (hasLoopCall)
    {
        if (!scanZone(BOX_MAIN_SLOTS, BOX_TOTAL_SLOTS, board, hasLoopCall))
        {
            Serial.println("[send] INVALID slot: aborting, nothing sent");
            runSendFeedback(runBuildErrorAnimationTest);
            return;
        }
    }
    else
    {
        Serial.println("[send] no LOOP_CALL in the main zone: loop zone not scanned");
    }

    built = buildProgramFromPieces(board, packet, isEmpty);
    if (!built)
    {
        if (isEmpty)
        {
            Serial.println("[send] empty board: no command to send");
            runSendFeedback(runEmptyBoardAnimationTest);
        }
        else
        {
            Serial.println("[send] build FAILED: invalid slot or payload overflow");
            Serial.println("[send] nothing sent");
            runSendFeedback(runBuildErrorAnimationTest);
        }
        return;
    }

    Serial.print("[send] program: ");
    Serial.print(static_cast<int>(packet.count));
    Serial.print(" command(s) [");
    for (uint8_t i = 0; i < packet.count; ++i)
    {
        if (i > 0)
        {
            Serial.print(' ');
        }
        Serial.print(static_cast<int>(packet.commands[i]));
    }
    Serial.println("]");

    /* The bench never listens: nothing puts the radio in RX, so
     * stopListening() states the intent before the burst. The car's ACK
     * comes back over the auto-ack channel and write() reports it, which
     * is the delivery proof the link validation needs */
    radio.stopListening();
    const bool sent = radio.write(&packet, sizeof(packet));
    Serial.print("[send] radio.write ");
    Serial.print(sizeof(packet));
    Serial.print(" bytes -> ");
    Serial.println(sent ? "ACK (car answered)" : "NO ACK (car absent or not listening)");

    if (sent)
    {
        runSendFeedback(runSuccessAnimationTest);
    }
    else
    {
        runSendFeedback(runRfErrorAnimationTest);
    }
}

/* ========================================== */
/* Radio menu, key "t"                        */
/* ========================================== */

/* RF24 v1.6 exposes no address getter, so the addresses printed are the
 * constants setup() wrote into the chip: that is the question when two
 * rigs do not hear each other. isChipConnected() does read the chip over
 * SPI, so entry 1 probes the hardware live instead of trusting the flag
 * setup() left behind */
static void printRadioAddress(const char *label, const uint8_t *addr)
{
    Serial.print(label);
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (addr[i] < 0x10U)
        {
            Serial.print('0');
        }
        Serial.print(addr[i], HEX);
        if (i < 4U)
        {
            Serial.print(' ');
        }
    }
    Serial.println();
}

static void printRadioMenu()
{
    Serial.println("   [radio] 1 probe | 2 addresses and config | 3 send test program | q quit");
}

void startRadioMenu()
{
    Serial.println("--- RADIO MENU ---");
    radio_menu_active = true;
    printRadioMenu();
}

static void stopRadioMenu()
{
    radio_menu_active = false;
    Serial.println("   [radio menu closed]");
}

void runRadioMenuKey(char key)
{
    switch (key)
    {
    case '1':
        Serial.print("[radio] chip ");
        Serial.println(radio.isChipConnected() ? "answering" : "NOT answering over SPI");
        break;

    case '2': {
        static const char *const kPaNames[] = {"MIN", "LOW", "HIGH", "MAX"};
        const uint8_t pa                    = radio.getPALevel();
        const rf24_datarate_e rate          = radio.getDataRate();

        printRadioAddress("   tx       = ", gcar_pipe_name);
        printRadioAddress("   rx pipe1 = ", gbox_pipe_name);
        Serial.print("   pa=");
        Serial.print((pa < 4U) ? kPaNames[pa] : "?");
        Serial.print(" rate=");
        switch (rate)
        {
        case RF24_1MBPS:
            Serial.print("1MBPS");
            break;
        case RF24_2MBPS:
            Serial.print("2MBPS");
            break;
        case RF24_250KBPS:
            Serial.print("250KBPS");
            break;
        default:
            Serial.print("?");
            break;
        }
        Serial.print(" channel=");
        Serial.print(radio.getChannel());
        Serial.print(" payload=");
        Serial.println(radio.getPayloadSize());
        break;
    }

    case '3': {
        /* A one command program proves the transmit path without a scanned
         * board: MOTOR_STOP is harmless whatever the car is doing */
        CommandPacket packet = {};
        bool sent            = false;

        if (!radio_ok)
        {
            Serial.println("[radio] radio is not up: re-probe with 's' in health mode");
            break;
        }

        packet.count       = 1;
        packet.commands[0] = static_cast<uint8_t>(CommandId::MOTOR_STOP);
        radio.stopListening();
        sent = radio.write(&packet, sizeof(packet));
        Serial.print("[radio] test program (MOTOR_STOP) -> ");
        Serial.println(sent ? "ACK (car answered)" : "NO ACK (car absent or not listening)");
        break;
    }

    case 'q':
        stopRadioMenu();
        return;

    default:
        Serial.println("   [radio] 1 probe | 2 addresses | 3 test send | q quit");
        return;
    }

    if (radio_menu_active)
    {
        printRadioMenu(); /* the entries are chatty: reprint the prompt after them */
    }
}

void runCarStreamTest()
{
    uint8_t buffer[sizeof(CommandPacket)];
    uint8_t logLine[64];
    uint8_t logLineLen       = 0;
    uint32_t packets         = 0;
    uint32_t lastHeartbeatMs = millis();

    /* Without this guard a dead radio looks exactly like a silent car:
     * both print "quiet, packets=0" until someone notices the car was
     * reporting FAILED all along */
    if (!radio_ok)
    {
        Serial.println("\n[stream] radio is not up: re-probe with 's' in health mode");
        return;
    }

    Serial.println("\n--- CAR STREAM on _box_ (telemetry + logs), any key to stop ---");
    /* State the address being listened on, so it can be read against the
     * sender's without opening either firmware's menu */
    printRadioAddress("   listening pipe1 = ", gbox_pipe_name);

    monitor_active = true;
    monitor_exit   = false;
    radio.startListening();

    while (!monitor_exit)
    {
        uint8_t pipe = 0;

        if (radio.available(&pipe))
        {
            radio.read(buffer, sizeof(buffer));
            ++packets;

            switch (static_cast<PacketTag>(buffer[0]))
            {
            case PacketTag::Telemetry: {
                TelemetryPacket telemetry;

                memcpy(&telemetry, buffer, sizeof(telemetry));
                Serial.print("   [TLM] yaw=");
                Serial.print(telemetry.yaw, 1);
                Serial.print(" tgt=");
                Serial.print(telemetry.targetYaw, 1);
                Serial.print(" state=");
                Serial.print(static_cast<int>(telemetry.state));
                Serial.print(" speed=");
                Serial.println(static_cast<int>(telemetry.speed));
                lastHeartbeatMs = millis();
                break;
            }
            case PacketTag::Log: {
                const uint8_t flags   = buffer[1];
                const uint8_t dataLen = flags & 0x7FU;
                const bool hasMore    = (flags & 0x80U) != 0U;
                const uint8_t room    = static_cast<uint8_t>(sizeof(logLine) - logLineLen);
                const uint8_t copy    = (dataLen < room) ? dataLen : room;

                memcpy(&logLine[logLineLen], &buffer[2], copy);
                logLineLen = static_cast<uint8_t>(logLineLen + copy);
                if (copy < dataLen)
                {
                    Serial.println("   [LOG] line too long, truncated");
                }
                if (hasMore)
                {
                    break;
                }

                Serial.print("   [LOG] ");
                Serial.write(logLine, logLineLen);
                Serial.println();
                logLineLen      = 0;
                lastHeartbeatMs = millis();
                break;
            }
            default:
                /* Byte 0 is the command count on a program and a real tag
                 * otherwise: reporting both the same way is misleading */
                if (buffer[0] > 0 && buffer[0] <= BOX_MAX_PAYLOAD_COMMANDS)
                {
                    Serial.print("   [PROG] ");
                    Serial.print(static_cast<int>(buffer[0]));
                    Serial.print(" command(s):");
                    for (uint8_t i = 0; i < buffer[0]; ++i)
                    {
                        Serial.print(' ');
                        Serial.print(commandIdToString(static_cast<CommandId>(buffer[1 + i])));
                    }
                    Serial.println();
                }
                else
                {
                    Serial.print("   [rx] unknown tag 0x");
                    Serial.println(buffer[0], HEX);
                }
                lastHeartbeatMs = millis();
                break;
            }
            continue;
        }

        /* A quiet stream must still look alive */
        if (millis() - lastHeartbeatMs >= 3000U)
        {
            lastHeartbeatMs = millis();
            Serial.print("   [stream] quiet, packets=");
            Serial.println(static_cast<int>(packets));
        }

        delayWithUsb(20);
    }

    monitor_active = false;
    radio.stopListening();

    Serial.print("--- CAR STREAM stopped, packets=");
    Serial.print(static_cast<int>(packets));
    Serial.println(" ---");
}

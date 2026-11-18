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

/* The stream is the only place where 'p' means "send the program": the slot
 * monitors share monitor_active and leave the radio alone */
bool car_stream_active         = false;
bool car_stream_send_requested = false;

BenchRadio::BenchRadio(rf24_gpio_pin_t cePin, rf24_gpio_pin_t csnPin, uint32_t spiSpeed)
    : RF24(cePin, csnPin, spiSpeed)
{
}

uint8_t BenchRadio::registerRead(uint8_t reg)
{
    return read_register(reg);
}

void BenchRadio::registerRead(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    read_register(reg, buffer, length);
}

/* printDetails() formats through printf and sprintfPrettyDetails() through
 * tabs and long lines, which the terminal wraps into a staircase. These are
 * the registers that decide whether a packet can be received at all, printed
 * compactly with Serial */
static void radio_print_registers()
{
    const uint8_t config  = radio.registerRead(nRF24L01::CONFIG);
    const uint8_t enabled = radio.registerRead(nRF24L01::EN_RXADDR);
    const uint8_t fifo    = radio.registerRead(nRF24L01::FIFO_STATUS);
    const uint8_t channel = radio.registerRead(nRF24L01::RF_CH);
    const uint8_t setup   = radio.registerRead(nRF24L01::RF_SETUP);
    uint8_t rx0[5]        = {};
    uint8_t rx1[5]        = {};
    uint8_t tx[5]         = {};

    radio.registerRead(nRF24L01::RX_ADDR_P0, rx0, sizeof(rx0));
    radio.registerRead(nRF24L01::RX_ADDR_P1, rx1, sizeof(rx1));
    radio.registerRead(nRF24L01::TX_ADDR, tx, sizeof(tx));

    Serial.print("   CONFIG=0x");
    Serial.print(config, HEX);
    Serial.print(" PWR_UP=");
    Serial.print((config >> 1) & 1U);
    Serial.print(" PRIM_RX=");
    Serial.print(config & 1U);
    Serial.print(" EN_RXADDR=0x");
    Serial.print(enabled, HEX);
    Serial.print(" FIFO=0x");
    Serial.print(fifo, HEX);
    Serial.print(" RF_CH=");
    Serial.print(channel);
    Serial.print(" RF_SETUP=0x");
    Serial.println(setup, HEX);

    Serial.print("   RX0=");
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (rx0[i] < 0x10U)
        {
            Serial.print('0');
        }
        Serial.print(rx0[i], HEX);
        Serial.print(' ');
    }
    Serial.print(" RX1=");
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (rx1[i] < 0x10U)
        {
            Serial.print('0');
        }
        Serial.print(rx1[i], HEX);
        Serial.print(' ');
    }
    Serial.print(" TX=");
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (tx[i] < 0x10U)
        {
            Serial.print('0');
        }
        Serial.print(tx[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

/* RF24's write() waits for a status flag with no timeout, so a chip that
 * stops answering over SPI freezes the whole bench: no prompt, no loop. The
 * non-blocking pair bounds that wait, and a dead radio reports a failure */
static bool radio_send(const void *data, uint8_t length)
{
    constexpr uint32_t TX_TIMEOUT_MS = 50;

    radio.stopListening();
    radio.startWrite(data, length, false);

    /* startWrite is a 32-byte burst: if it did not reach the FIFO, txStandBy
     * sees an empty queue and reports success, and the bench would wait for an
     * answer to a packet that never left. Check the queue was really loaded */
    if ((radio.registerRead(nRF24L01::FIFO_STATUS) & (1U << nRF24L01::TX_EMPTY)) != 0U)
    {
        radio.startListening();
        return false;
    }

    const bool sent = radio.txStandBy(TX_TIMEOUT_MS, true);
    radio.startListening();
    return sent;
}

/* Bring the chip up from scratch and put it in the production state. Called
 * at boot and again by the health cycle: a failed probe at boot must not
 * leave the bench unable to test the link without a reset, and the two
 * paths must not set the chip differently */
bool radio_begin()
{
    if (!radio.begin())
    {
        radio_ok = false;
        Serial.println("   [ERROR] nRF24L01+ not found!");
        Serial.println("   -> Check soldering on PA3, PA4, PA5, PA6, PA7.");
        Serial.println("   -> Check if 3.3V is reaching the nRF board.");
        Serial.println("   -> 'p' and 'e' stay disabled until it answers.");
        return false;
    }

    Serial.println("   [SUCCESS] nRF24L01+ detected! SPI is working.");
    /* RF_SETUP can come back from a test with CONT_WAVE and PLL_LOCK set, and
     * the module keeps them across MCU resets because its 3.3 V never drops.
     * In that mode the chip answers SPI and transmits a carrier instead of
     * packets, so the link goes silent with no other symptom. The library
     * keeps write_register() private, but this is exactly the call it offers
     * to leave that mode */
    radio.stopConstCarrier();
    /* Same line as production RadioController::begin(): the bench must put the
     * chip in the exact state the GBox will, or the link it validates is not
     * the link production will run */
    radio.setPALevel(PA_LEVEL);
    radio.setDataRate(DATA_RATE);
    radio.setAddressWidth(5);
    radio.setPayloadSize(sizeof(CommandPacket));
    radio.openWritingPipe(gcar_pipe_name); /* the GBox writes to the car */
    /* The real box layout: pipe 0 carries the program burst to _car_, pipe 1
     * listens on _box_ where the car answers with telemetry and logs. The
     * shell drains pipe 1 between keys: a full FIFO stops the chip from
     * acknowledging, and the car then waits on every acknowledged send */
    radio.openReadingPipe(1, gbox_pipe_name);
    /* stopConstCarrier() powers the chip down per the datasheet, so bring it
     * back up explicitly and leave it listening: the probe must end in the
     * state the bench runs in, not in one the first idle slice repairs */
    radio.startListening();
    radio_print_registers();
    radio_ok = true;
    return true;
}

/* Keep the other side's pipe from filling. The chip only acknowledges a
 * packet when the RX FIFO has room, and a bench that never reads makes the
 * car's acknowledged sends wait for retries that will never succeed. Outside
 * the 'e' stream nothing displays these packets, so they are read and
 * dropped; the stream does its own reading */
void radio_discard_incoming()
{
    uint8_t payload[sizeof(CommandPacket)];
    uint8_t pipe = 0;

    if (!radio_ok || car_stream_active)
    {
        return;
    }

    /* A send or the stream menu may have left the chip in standby. Listening
     * is idempotent, and doing it every shell slice costs two register
     * writes */
    radio.startListening();

    while (radio.available(&pipe))
    {
        radio.read(payload, sizeof(payload));
    }
}

/* Ask the car which build it runs. The write answers whether the radio chip
 * heard us; the version answer, whether the application is alive. Both are
 * useful apart: a hardware ACK with no answer means the car is stuck or runs
 * a firmware without the handler */
void radio_run_ping()
{
    constexpr uint32_t PING_TIMEOUT_MS = 1000;

    VersionPacket request                 = {};
    VersionPacket answer                  = {};
    uint8_t buffer[sizeof(VersionPacket)] = {};
    bool answered                         = false;

    if (!radio_ok)
    {
        Serial.println("[ping] radio is not up: re-probe with 's' in health mode");
        return;
    }

    request.tag = static_cast<uint8_t>(PacketTag::VersionRequest);

    const uint32_t started = millis();
    const bool sent        = radio_send(&request, sizeof(request));

    if (!sent)
    {
        Serial.println("[ping] no ACK: the car is absent or not listening");
        return;
    }

    while (millis() - started < PING_TIMEOUT_MS)
    {
        uint8_t pipe = 0;

        if (radio.available(&pipe))
        {
            radio.read(buffer, sizeof(buffer));
            if (buffer[0] == static_cast<uint8_t>(PacketTag::Version))
            {
                memcpy(&answer, buffer, sizeof(answer));
                answered = true;
                break;
            }
        }
    }

    const uint32_t elapsed = millis() - started;

    if (!answered)
    {
        Serial.println("[ping] ACKed but no answer: the application is silent");
        return;
    }

    Serial.print("[ping] build ");
    Serial.print(static_cast<int>(answer.build));
    Serial.print(" (");
    Serial.print(answer.version);
    Serial.print("), wire protocol v");
    Serial.print(static_cast<int>(answer.wire));
    Serial.print(", pong in ");
    Serial.print(elapsed);
    Serial.println(" ms");
}

/* ========================================== */
/* Send the scanned board over the radio, key "p" */
/* ========================================== */

/* The LED verdict only shows if the 5V rail is up; calibration keeps it
 * down, so raise it for the animation and put it back, the same contract
 * the LED menu uses */
static void radio_run_send_feedback(void (*animation)())
{
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);
    delay(RAIL_SETTLE_DELAY_MS);
    animation();
    leds_blackout();
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
}

/* Scan -> build -> transmit, the chain production GBox runs when it
 * validates a program, without GBox itself: the decode is the compiled
 * per-slot row (scan_piece_for_raw, the one walk), the build is the shared
 * buildProgramFromPieces, only the transport is this bench. The verdict
 * mirrors the production LED semantics: green on ACK, amber on an empty
 * board, red on a refused build, RF error when the car does not answer */
void radio_run_send_pieces_test()
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
    if (!scan_zone(0, BOX_MAIN_SLOTS, board, hasLoopCall))
    {
        Serial.println("[send] INVALID slot: aborting, nothing sent");
        radio_run_send_feedback(leds_run_build_error_animation_test);
        return;
    }

    /* The loop zone is read only when the main zone held a LOOP_CALL:
     * without one those slots never reach the program */
    if (hasLoopCall)
    {
        if (!scan_zone(BOX_MAIN_SLOTS, BOX_TOTAL_SLOTS, board, hasLoopCall))
        {
            Serial.println("[send] INVALID slot: aborting, nothing sent");
            radio_run_send_feedback(leds_run_build_error_animation_test);
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
            radio_run_send_feedback(leds_run_empty_board_animation_test);
        }
        else
        {
            Serial.println("[send] build FAILED: invalid slot or payload overflow");
            Serial.println("[send] nothing sent");
            radio_run_send_feedback(leds_run_build_error_animation_test);
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
    const bool sent = radio_send(&packet, sizeof(packet));
    Serial.print("[send] radio.write ");
    Serial.print(sizeof(packet));
    Serial.print(" bytes -> ");
    Serial.println(sent ? "ACK (car answered)" : "NO ACK (car absent or not listening)");

    if (sent)
    {
        radio_run_send_feedback(leds_run_success_animation_test);
    }
    else
    {
        radio_run_send_feedback(leds_run_rf_error_animation_test);
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
static void radio_print_address(const char *label, const uint8_t *addr)
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

static void radio_print_menu()
{
    Serial.println("   [radio] 1 probe | 2 addresses and config | 3 send test program | q quit");
}

void radio_start_menu()
{
    Serial.println("--- RADIO MENU ---");
    radio_menu_active = true;
    radio_print_menu();
}

static void radio_stop_menu()
{
    radio_menu_active = false;
    Serial.println("   [radio menu closed]");
}

void radio_run_menu_key(char key)
{
    switch (key)
    {
    case '1':
        Serial.print("[radio] chip ");
        Serial.println(radio.isChipConnected() ? "answering" : "NOT answering over SPI");
        break;

    case '2': {
        static const char *const PA_NAMES[] = {"MIN", "LOW", "HIGH", "MAX"};
        const uint8_t pa                    = radio.getPALevel();
        const rf24_datarate_e rate          = radio.getDataRate();

        radio_print_address("   tx       = ", gcar_pipe_name);
        radio_print_address("   rx pipe1 = ", gbox_pipe_name);
        Serial.print("   pa=");
        Serial.print((pa < 4U) ? PA_NAMES[pa] : "?");
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
        sent               = radio_send(&packet, sizeof(packet));
        Serial.print("[radio] test program (MOTOR_STOP) -> ");
        Serial.println(sent ? "ACK (car answered)" : "NO ACK (car absent or not listening)");
        break;
    }

    case 'q':
        radio_stop_menu();
        return;

    default:
        Serial.println("   [radio] 1 probe | 2 addresses | 3 test send | q quit");
        return;
    }

    if (radio_menu_active)
    {
        radio_print_menu(); /* the entries are chatty: reprint the prompt after them */
    }
}

void radio_run_car_stream_test()
{
    uint8_t buffer[sizeof(CommandPacket)];
    /* The car's longest line is 80 bytes, and the relay sends 30 per packet:
     * a 64 byte buffer cut them and printed "line too long" */
    uint8_t logLine[96];
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

    Serial.println("\n--- CAR STREAM on _box_ (telemetry + logs), any key stops, p sends ---");
    /* State the address being listened on, so it can be read against the
     * sender's without opening either firmware's menu */
    radio_print_address("   listening pipe1 = ", gbox_pipe_name);

    monitor_active            = true;
    monitor_exit              = false;
    car_stream_active         = true;
    car_stream_send_requested = false;
    radio.startListening();
    radio_print_registers();

    while (!monitor_exit)
    {
        /* 'p' only sets the request: the send happens here, between two
         * reads, so the program is never built while the loop is inside the
         * chip's RX path. startListening() after it is mandatory, the send
         * ends with stopListening() and the stream would go deaf */
        if (car_stream_send_requested)
        {
            radio_run_send_pieces_test();
            radio.startListening();
            car_stream_send_requested = false;
            /* The stream was deaf for the whole scan and animation, so the
             * half line being reassembled has lost its middle: dropping it
             * stops the tail gluing itself to the next line */
            logLineLen = 0;
        }

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

        shell_delay_with_usb(20);
    }

    monitor_active    = false;
    car_stream_active = false;
    radio.stopListening();

    Serial.print("--- CAR STREAM stopped, packets=");
    Serial.print(static_cast<int>(packets));
    Serial.println(" ---");
}

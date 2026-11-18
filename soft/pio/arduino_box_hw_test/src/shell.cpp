/* USB key shell: the help text, character echo and the command
 * dispatcher with its dfu matcher and digit-prefix state */
#include "FirmwareVersion.h"
#include "WireVersion.h"
#include "dfu_bootloader.h"
#include "hwtest.h"

#ifdef TEST_ADS1115
/* The key list, shown at boot and on demand by '?'. One line per concern,
 * commands with their slot prefix spelled out as real examples */
void shell_print_usb_help()
{
    Serial.println("   modes: h = health | c = calibration | s = rerun test | ? = this help");
    Serial.println("   slots: 5m = live monitor | 5x = frozen monitor: SPACE = clean 3s");
    Serial.println("          measure, any other key quits");
    jig_print_help();
    Serial.println("   pieces: e empty, f forward, L loop, r turn right, l turn left");
    Serial.println("   rows:  w = reprint analysis and pasteable table");
    Serial.println("   leds:  l = LED test menu (walk, per-LED channels, animations; q quits)");
    Serial.println("   flash: type dfu to jump to the bootloader (reflash or power-cycle to exit)");
    Serial.println("   build: v = show the build this bench was flashed from");
    Serial.println("   reset: type reset to reboot the MCU (five letters in a row)");
    Serial.println("   power:  b = deep sleep (STOP): only the wake button wakes the chip");
    Serial.println("   radio:  p = scan the 20 slots, build the program and send it to the car");
    Serial.println(
        "           green = ACK, amber = empty board, red = refused, RF error = no answer");
    Serial.println("           t = radio menu: 1 probe, 2 addresses, 3 test send, q quits");
    Serial.println("   stream: e = listen to _box_ for the car telemetry and logs");
    Serial.println("           p inside that stream = scan, build and send; any other key quits");
}

/* Sliced delay so USB commands keep being polled while a test cycle waits.
 * Rail settle delays keep the plain delay(): the electrical settling must
 * not be interrupted */
void shell_delay_with_usb(uint32_t totalMs)
{
    constexpr uint32_t SLICE_MS = 50;
    uint32_t remaining          = totalMs;
    while (remaining > 0)
    {
        const uint32_t slice = remaining > SLICE_MS ? SLICE_MS : remaining;
        delay(slice);
        remaining -= slice;
        shell_handle_usb_commands();
    }
}

/* Identity of the firmware in flash: the shell runs on the board, so this is
 * the build that is really there, not the one in the editor */
static void shell_print_version()
{
    Serial.print("   hw-test build ");
    Serial.print(FW_VERSION);
    Serial.print(" (");
    Serial.print(FW_BUILD);
    Serial.print("), wire protocol v");
    Serial.println(static_cast<int>(kWireProtocolVersion));
}

/* Mode switch for the h/c keys: both rails down, the calibration stats
 * cleared on entry, and the sensor rail handed back to the per-read lease */
void shell_enter_mode(TestMode mode)
{
    if (test_mode == mode)
    {
        run_test_requested = true; /* already active: re-run this mode test */
        Serial.println("\n-> test already active, running it ('?' status)");
        return;
    }

    test_mode = mode;
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    if (test_mode == TestMode::Calibration)
    {
        ads_reset_calibration_stats();
        /* The sensor rail goes up per reading now, so entering the mode only
         * takes it down: holding it up for the whole session is what warmed
         * the board and shifted every reading by ~370 counts */
        calibration_rails_on = false;
        Serial.println("\n-> mode: calibration ('s' scan, 't' gaps, '?' status)");
    }
    else
    {
        calibration_rails_on = false;
        Serial.println("\n-> mode: health ('?' for status)");
    }
}

/* Terminal-style echo of typed keys: serial monitors do not echo locally,
 * so without this the operator types blind. Printable characters print as
 * they arrive and build the line under the fingers; SPACE is a command key
 * with no visible glyph so it marks itself; CR is dropped because LF is the
 * line end (the same convention the import parser follows) */
static void shell_echo_typed_key(int incoming)
{
    if (incoming == '\n')
    {
        Serial.println();
        return;
    }
    if (incoming == ' ')
    {
        Serial.print("[SP]");
        return;
    }
    if (incoming >= '!' && incoming <= '~')
    {
        Serial.print(static_cast<char>(incoming));
    }
}

void shell_handle_usb_commands()
{
    /* pendingSlot collects a typed slot number that prefixes a slot command
     * ('5m', '5R'): digits accumulate in a row and the next command key ends
     * the selection; any other key clears the pending digits */
    static uint16_t pendingSlot = 0;
    static bool digitsStarted   = false;

    while (Serial.available() > 0)
    {
        const int incoming = Serial.read();

        shell_echo_typed_key(incoming);

        /* While a monitor runs, any key is a stop request, except 'p' inside
         * the car stream: there it sends the scanned program and the monitor
         * keeps running. The slot monitors leave 'p' alone */
        if (monitor_active)
        {
            if (car_stream_active && incoming == 'p')
            {
                car_stream_send_requested = true;
                continue;
            }

            monitor_exit  = true;
            pendingSlot   = 0;
            digitsStarted = false;
            /* Drop the rest of the burst: this stop key cuts a command typed
             * while the monitor ran, and the surviving tail would decode as a
             * different command. The operator retypes it once the monitor has
             * unwound, and the count makes the eaten characters visible */
            int dropped = 0;
            while (Serial.available() > 0)
            {
                Serial.read();
                ++dropped;
            }
            if (dropped > 0)
            {
                Serial.print(" [dropped ");
                Serial.print(dropped);
                Serial.println("]");
            }
            break;
        }

        /* Frozen slot monitor: SPACE arms one measurement window (swallowed
         * while a window already runs: one key, one measure), trailing
         * newlines are harmless, any other key quits with the same burst
         * drain as the live monitor */
        if (freeze_active)
        {
            if (incoming == ' ')
            {
                if (!freeze_measuring)
                {
                    freeze_measure = true;
                }
            }
            else if (incoming != '\r' && incoming != '\n')
            {
                freeze_exit = true;
                int dropped = 0;
                while (Serial.available() > 0)
                {
                    Serial.read();
                    ++dropped;
                }
                if (dropped > 0)
                {
                    Serial.print(" [dropped ");
                    Serial.print(dropped);
                    Serial.println("]");
                }
            }
            pendingSlot   = 0;
            digitsStarted = false;
            if (freeze_exit)
            {
                break;
            }
            continue;
        }

        if (led_menu_active)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                leds_run_test_key(static_cast<char>(incoming));
            }
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        if (radio_menu_active)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                radio_run_menu_key(static_cast<char>(incoming));
            }
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        /* Jig mode intercepts its own keys, like the game does: the walk is
         * driven by the polling loop and only the escape hatches are here */
        if (jig_active)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                jig_run_key(static_cast<char>(incoming));
            }
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        if (incoming >= '0' && incoming <= '9')
        {
            pendingSlot   = static_cast<uint16_t>(pendingSlot * 10 + (incoming - '0'));
            digitsStarted = true;
            continue;
        }

        /* Typing reset (five letters in a row) reboots the MCU, same
         * matcher style as dfu above; the letters are consumed only while
         * the word still matches, any other character breaks the sequence
         * and is handled normally. NVIC_SystemReset never returns: it goes
         * through the full boot path (rails, APA106 dark frame, menus) */
        static const char RESET_WORD[] = "reset";
        static uint8_t resetWordSeen   = 0;
        if (incoming == RESET_WORD[resetWordSeen])
        {
            if (++resetWordSeen == sizeof(RESET_WORD) - 1U)
            {
                resetWordSeen = 0;
                Serial.println("\n--- MCU RESET ---");
                delay(50); /* let the line flush before the USB link drops */
                NVIC_SystemReset();
            }
            continue;
        }
        resetWordSeen = 0;

        /* Typing dfu (three letters in a row) enters the DFU bootloader.
         * The letters are consumed only while the word still matches: any
         * other character breaks the sequence and is handled normally. The
         * shared enterDfuBootloader() never returns */
        static const char DFU_WORD[] = "dfu";
        static uint8_t dfuWordSeen   = 0;
        if (incoming == DFU_WORD[dfuWordSeen])
        {
            if (++dfuWordSeen == sizeof(DFU_WORD) - 1U)
            {
                dfuWordSeen = 0;
                Serial.println("\n--- DFU: reflash or power-cycle to exit ---");
                enterDfuBootloader();
            }
            continue;
        }
        dfuWordSeen = 0;

        switch (incoming)
        {
        case 'h':
            shell_enter_mode(TestMode::Health);
            break;
        case 'c':
            shell_enter_mode(TestMode::Calibration);
            break;
        case 's':
            run_test_requested = true;
            break;
        case '?':
            Serial.print("mode: ");
            Serial.println(test_mode == TestMode::Health ? "health" : "calibration");
            shell_print_usb_help();
            break;
        case 'v':
            shell_print_version();
            break;
        case 'l':
            if (led_menu_active)
            {
                break; /* unreachable: the menu intercepts before the switch */
            }
            leds_start_test_menu();
            break;

        case 't':
            if (radio_menu_active)
            {
                break; /* unreachable: the menu intercepts before the switch */
            }
            radio_start_menu();
            break;
        case 'b':
            power_run_deep_sleep_test();
            break;
        case 'p':
            radio_run_send_pieces_test();
            break;
        case 'e':
            radio_run_car_stream_test();
            break;
        case 'j':
            if (jig_active)
            {
                jig_stop_mode();
                break;
            }
            jig_start_mode();
            break;
        case 'k':
            if (test_mode != TestMode::Calibration)
            {
                Serial.println("forgetting a slot works in calibration mode ('c' to switch)");
                break;
            }
            if (!digitsStarted || pendingSlot < 1 || pendingSlot > CALIBRATION_SLOT_COUNT)
            {
                Serial.println("prefix the slot number: 19k forgets slot 19");
                break;
            }
            jig_forget_slot(static_cast<uint8_t>(pendingSlot - 1));
            break;
        case 'm':
            if (test_mode != TestMode::Calibration)
            {
                Serial.println("slot monitor works in calibration mode ('c' to switch)");
                break;
            }
            if (!digitsStarted || pendingSlot < 1 || pendingSlot > CALIBRATION_SLOT_COUNT)
            {
                Serial.println("type '<slot>m' e.g. '5m' (monitor slot 5)");
                break;
            }
            scan_run_slot_monitor(static_cast<uint8_t>(pendingSlot - 1));
            break;
        case 'x':
            if (test_mode != TestMode::Calibration)
            {
                Serial.println("frozen monitor works in calibration mode ('c' to switch)");
                break;
            }
            if (!digitsStarted || pendingSlot < 1 || pendingSlot > CALIBRATION_SLOT_COUNT)
            {
                Serial.println("type '<slot>x' e.g. '5x' (frozen monitor slot 5)");
                break;
            }
            scan_run_frozen_slot_monitor(static_cast<uint8_t>(pendingSlot - 1));
            break;
        default:
            break;
        }

        digitsStarted = false;
        pendingSlot   = 0;
    }
}
#endif /* TEST_ADS1115 */

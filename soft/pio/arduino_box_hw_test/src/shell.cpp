/* USB key shell: context prompt, help text, character echo and the command
 * dispatcher */
#include "FirmwareVersion.h"
#include "WireVersion.h"
#include "dfu_bootloader.h"
#include "hwtest.h"

#ifdef TEST_ADS1115
/* The key list, shown on demand by '?'. Short and grouped: the boot only
 * prints a hint and the prompt, so the console stays readable */
void shell_print_usb_help()
{
    /* Only the keys that mean something here: a health-mode list that offers
     * the slot monitors is what made the old shell feel ambiguous */
    if (test_mode == TestMode::Health)
    {
        Serial.println("   health mode");
        Serial.println("   h rerun cycle | c calibration | s health cycle");
        Serial.println("   e stream | r ping | l leds | t radio | v version");
        Serial.println("   b deep sleep | D DFU bootloader | R reset | ? help");
    }
    else
    {
        Serial.println("   calibration mode");
        Serial.println("   c scan | s scan | h health");
        Serial.println("   m live monitor | f frozen | k forget | j campaign");
        Serial.println("   p send the board | e stream | r ping | l leds | t radio");
        Serial.println("   v version | b deep sleep | D DFU bootloader | R reset | ? help");
    }
}

/* The prompt says which context consumes the next key, so a key that changes
 * meaning inside a menu is never a surprise */
void shell_print_prompt()
{
    const char *mode = (test_mode == TestMode::Health) ? "health" : "calibration";

    if (led_menu_active)
    {
        Serial.print(mode);
        Serial.print("/leds> ");
    }
    else if (radio_menu_active)
    {
        Serial.print(mode);
        Serial.print("/radio> ");
    }
    else if (jig_active)
    {
        Serial.print(mode);
        Serial.print("/jig> ");
    }
    else
    {
        Serial.print(mode);
        Serial.print("> ");
    }
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
        radio_discard_incoming();
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

/* Ask for a slot on its own line. One rule instead of a digit prefix: the
 * command key comes first and the number follows, so there is no syntax to
 * remember and no stale digits to clear */
static uint8_t shell_ask_slot(const char *action)
{
    constexpr uint32_t ENTRY_TIMEOUT_MS = 10000;

    uint16_t value         = 0;
    bool any               = false;
    const uint32_t started = millis();

    Serial.print("   ");
    Serial.print(action);
    Serial.print(": slot 1-");
    Serial.print(static_cast<int>(CALIBRATION_SLOT_COUNT));
    Serial.print(" (0 cancels) ? ");

    while (millis() - started < ENTRY_TIMEOUT_MS)
    {
        if (Serial.available() <= 0)
        {
            delay(1);
            continue;
        }

        const int incoming = Serial.read();

        if (incoming == '\r')
        {
            continue;
        }
        if (incoming == '\n')
        {
            Serial.println();
            break;
        }
        if (incoming >= '0' && incoming <= '9')
        {
            value = static_cast<uint16_t>(value * 10 + (incoming - '0'));
            any   = true;
            Serial.print(static_cast<char>(incoming));
            continue;
        }

        Serial.println(" [cancelled]");
        return 0;
    }

    if (!any || value < 1 || value > CALIBRATION_SLOT_COUNT)
    {
        Serial.println("   no slot given");
        return 0;
    }

    return static_cast<uint8_t>(value);
}

/* Mode switch for the h/c keys: both rails down, the calibration stats
 * cleared on entry, and the sensor rail handed back to the per-read lease */
void shell_enter_mode(TestMode mode)
{
    if (test_mode == mode)
    {
        /* Run it now, not through the loop: the prompt is printed when this
         * returns, so a deferred run would print its output after the prompt */
        Serial.println("   test already active, running it");
        if (mode == TestMode::Calibration)
        {
            scan_run_piece_calibration();
        }
        else
        {
            health_run_cycle();
        }
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
        Serial.println("   mode: calibration ('s' scan, 'm' monitor)");
    }
    else
    {
        calibration_rails_on = false;
        Serial.println("   mode: health");
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

            if (incoming != '\n' && incoming != '\r')
            {
                Serial.println();
            }
            monitor_exit = true;
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
                Serial.println();
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
                Serial.println();
                leds_run_test_key(static_cast<char>(incoming));
            }
            shell_print_prompt();
            continue;
        }

        if (radio_menu_active)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                Serial.println();
                radio_run_menu_key(static_cast<char>(incoming));
            }
            shell_print_prompt();
            continue;
        }

        /* Jig mode intercepts its own keys, like the game does: the walk is
         * driven by the polling loop and only the escape hatches are here */
        if (jig_active)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                Serial.println();
                jig_run_key(static_cast<char>(incoming));
            }
            shell_print_prompt();
            continue;
        }

        /* Dangerous actions are uppercase so they never collide with a
         * single-key command. Lowercase 'r' pings the car: with the old
         * word matcher it was swallowed as the first letter of "reset" */
        if (incoming != '\n' && incoming != '\r')
        {
            Serial.println();
        }

        switch (incoming)
        {
        case 'h':
            shell_enter_mode(TestMode::Health);
            break;
        case 'c':
            shell_enter_mode(TestMode::Calibration);
            break;
        case 's':
            if (test_mode == TestMode::Calibration)
            {
                scan_run_piece_calibration();
            }
            else
            {
                health_run_cycle();
            }
            break;
        case '?':
            shell_print_usb_help();
            break;
        case 'v':
            shell_print_version();
            break;
        case 'l':
            leds_start_test_menu();
            break;
        case 't':
            radio_start_menu();
            break;
        case 'b':
            power_run_deep_sleep_test();
            break;
        case 'p':
            radio_run_send_pieces_test();
            break;
        case 'r':
            radio_run_ping();
            break;
        case 'e':
            radio_run_car_stream_test();
            break;
        case 'j':
            if (test_mode != TestMode::Calibration)
            {
                Serial.println("   the jig campaign runs in calibration mode ('c')");
                break;
            }
            jig_start_mode();
            break;
        case 'm': {
            if (test_mode != TestMode::Calibration)
            {
                Serial.println("   slot monitors work in calibration mode ('c')");
                break;
            }
            const uint8_t slot = shell_ask_slot("live monitor");
            if (slot != 0)
            {
                scan_run_slot_monitor(static_cast<uint8_t>(slot - 1));
            }
            break;
        }
        case 'f': {
            if (test_mode != TestMode::Calibration)
            {
                Serial.println("   slot monitors work in calibration mode ('c')");
                break;
            }
            const uint8_t slot = shell_ask_slot("frozen monitor");
            if (slot != 0)
            {
                scan_run_frozen_slot_monitor(static_cast<uint8_t>(slot - 1));
            }
            break;
        }
        case 'k': {
            if (test_mode != TestMode::Calibration)
            {
                Serial.println("   forgetting a slot works in calibration mode ('c')");
                break;
            }
            const uint8_t slot = shell_ask_slot("forget");
            if (slot != 0)
            {
                jig_forget_slot(static_cast<uint8_t>(slot - 1));
            }
            break;
        }
        case 'D':
            Serial.println("   DFU: reflash or power-cycle to exit");
            enterDfuBootloader();
            break;
        case 'R':
            Serial.println("   MCU RESET");
            delay(50); /* let the line flush before the USB link drops */
            NVIC_SystemReset();
            break;
        default:
            break;
        }

        shell_print_prompt();
    }
}
#endif /* TEST_ADS1115 */

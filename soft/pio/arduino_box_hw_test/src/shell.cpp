/* USB key shell: the help text, character echo and the command
 * dispatcher with its dfu matcher and digit-prefix state */
#include "dfu_bootloader.h"
#include "hwtest.h"

#ifdef TEST_ADS1115
/* The key list, shown at boot and on demand by '?'. One line per concern,
 * commands with their slot prefix spelled out as real examples */
void printUsbHelp()
{
    Serial.println("   modes: h = health | c = calibration | s = rerun test | ? = this help");
    Serial.println("   slots: 5m = live monitor | 5x = frozen monitor: SPACE = clean 3s");
    Serial.println("          measure, any other key quits");
    printJigHelp();
    Serial.println("   pieces: e empty, f forward, L loop, r turn right, l turn left");
    Serial.println("   rows:  w = reprint analysis and pasteable table");
    Serial.println("   leds:  l = LED test menu (walk, per-LED channels, animations; q quits)");
    Serial.println("   flash: type dfu to jump to the bootloader (reflash or power-cycle to exit)");
    Serial.println("   reset: type reset to reboot the MCU (five letters in a row)");
    Serial.println("   power:  b = deep sleep (STOP): only the wake button wakes the chip");
    Serial.println("   radio:  p = scan the 20 slots, build the program and send it to the car");
    Serial.println(
        "           green = ACK, amber = empty board, red = refused, RF error = no answer");
    Serial.println("           t = radio menu: 1 probe, 2 addresses, 3 test send, q quits");
    Serial.println("   stream: e = listen to _box_ for the car telemetry and logs, key quits");
}

/* Terminal-style echo of typed keys: serial monitors do not echo locally,
 * so without this the operator types blind. Printable characters print as
 * they arrive and build the line under the fingers; SPACE is a command key
 * with no visible glyph so it marks itself; CR is dropped because LF is the
 * line end (the same convention the import parser follows) */
static void echoTypedKey(int incoming)
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

void handleUsbCommands()
{
    /* pendingSlot collects a typed slot number that prefixes a slot command
     * ('5m', '5R'): digits accumulate in a row and the next command key ends
     * the selection; any other key clears the pending digits */
    static uint16_t pendingSlot = 0;
    static bool digitsStarted   = false;

    while (Serial.available() > 0)
    {
        const int incoming = Serial.read();

        echoTypedKey(incoming);

        /* While the slot monitor runs, any key is a stop request */
        if (gMonitorActive)
        {
            gMonitorExit  = true;
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
        if (gFreezeActive)
        {
            if (incoming == ' ')
            {
                if (!gFreezeMeasuring)
                {
                    gFreezeMeasure = true;
                }
            }
            else if (incoming != '\r' && incoming != '\n')
            {
                gFreezeExit = true;
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
            if (gFreezeExit)
            {
                break;
            }
            continue;
        }

        if (gLedMenuActive)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                runLedTestKey(static_cast<char>(incoming));
            }
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        if (gRadioMenuActive)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                runRadioMenuKey(static_cast<char>(incoming));
            }
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        /* Jig mode intercepts its own keys, like the game does: the walk is
         * driven by the polling loop and only the escape hatches are here */
        if (gJigActive)
        {
            if (incoming != '\n' && incoming != '\r')
            {
                runJigKey(static_cast<char>(incoming));
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
        static const char kResetWord[] = "reset";
        static uint8_t resetWordSeen   = 0;
        if (incoming == kResetWord[resetWordSeen])
        {
            if (++resetWordSeen == sizeof(kResetWord) - 1U)
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
        static const char kDfuWord[] = "dfu";
        static uint8_t dfuWordSeen   = 0;
        if (incoming == kDfuWord[dfuWordSeen])
        {
            if (++dfuWordSeen == sizeof(kDfuWord) - 1U)
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
            enterMode(TestMode::Health);
            break;
        case 'c':
            enterMode(TestMode::Calibration);
            break;
        case 's':
            gRunTestRequested = true;
            break;
        case '?':
            Serial.print("mode: ");
            Serial.println(gMode == TestMode::Health ? "health" : "calibration");
            printUsbHelp();
            break;
        case 'l':
            if (gLedMenuActive)
            {
                break; /* unreachable: the menu intercepts before the switch */
            }
            startLedTestMenu();
            break;

        case 't':
            if (gRadioMenuActive)
            {
                break; /* unreachable: the menu intercepts before the switch */
            }
            startRadioMenu();
            break;

        case 'b':
            runDeepSleepTest();
            break;
        case 'p':
            runSendPiecesTest();
            break;
        case 'e':
            runCarStreamTest();
            break;
        case 'j':
            if (gJigActive)
            {
                stopJigMode();
                break;
            }
            startJigMode();
            break;
        case 'k':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("forgetting a slot works in calibration mode ('c' to switch)");
                break;
            }
            if (!digitsStarted || pendingSlot < 1 || pendingSlot > kCalibrationSlotCount)
            {
                Serial.println("prefix the slot number: 19k forgets slot 19");
                break;
            }
            forgetJigSlot(static_cast<uint8_t>(pendingSlot - 1));
            break;
        case 'm':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("slot monitor works in calibration mode ('c' to switch)");
                break;
            }
            if (!digitsStarted || pendingSlot < 1 || pendingSlot > kCalibrationSlotCount)
            {
                Serial.println("type '<slot>m' e.g. '5m' (monitor slot 5)");
                break;
            }
            runSlotMonitor(static_cast<uint8_t>(pendingSlot - 1));
            break;
        case 'x':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("frozen monitor works in calibration mode ('c' to switch)");
                break;
            }
            if (!digitsStarted || pendingSlot < 1 || pendingSlot > kCalibrationSlotCount)
            {
                Serial.println("type '<slot>x' e.g. '5x' (frozen monitor slot 5)");
                break;
            }
            runFrozenSlotMonitor(static_cast<uint8_t>(pendingSlot - 1));
            break;
        default:
            break;
        }

        digitsStarted = false;
        pendingSlot   = 0;
    }
}
#endif /* TEST_ADS1115 */

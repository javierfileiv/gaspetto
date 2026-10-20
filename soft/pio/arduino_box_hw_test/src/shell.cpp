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
    Serial.println("          measure, any other key quits | 5R = clear slot 5 samples");
    Serial.println("   game:  g = full campaign | 7g = slot 7 only | 7gf = slot 7 FORWARD only");
    Serial.println("          in game: SPACE record, a toggle auto-capture, r retry, q quit+emit");
    Serial.println("   pieces: e empty, f forward, L loop, r turn right, l turn left");
    Serial.println("   rows:  w = reprint analysis and pasteable table | D = dump clusters");
    Serial.println("   restore: i = paste a D dump, end with a line holding only .");
    Serial.println("   flash: type dfu to jump to the bootloader (reflash or power-cycle to exit)");
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

    /* Armed scoped start: '<slot>g' waits one more key for the piece
     * letter before launching, so it must survive the Enter that a
     * line-buffered terminal appends */
    static bool gameArmed    = false;
    static uint8_t armedSlot = 0;

    while (Serial.available() > 0)
    {
        const int incoming = Serial.read();

        /* Restore mode wins over every key handler: it consumes the raw
         * text stream instead of interpreting single keys, and echoes whole
         * lines instead of characters (a paste would drown the screen) */
        if (gClusterImportActive)
        {
            handleClusterImportChar(static_cast<char>(incoming));
            pendingSlot   = 0;
            digitsStarted = false;
            gameArmed     = false;
            continue;
        }

        echoTypedKey(incoming);

        /* While the slot monitor runs, any key is a stop request */
        if (gMonitorActive)
        {
            gMonitorExit  = true;
            pendingSlot   = 0;
            digitsStarted = false;
            gameArmed     = false;
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

        /* Armed scoped start: the key after '<slot>g' decides. A piece
         * letter scopes the campaign to that slot and piece, a second 'g'
         * walks every entered piece of the slot; line endings keep the
         * state alive, anything else cancels and is handled normally */
        if (gameArmed)
        {
            if (incoming == '\n' || incoming == '\r')
            {
                continue;
            }
            gameArmed = false;
            if (incoming == 'g')
            {
                startGameScoped(armedSlot, -1);
            }
            else
            {
                const int pieceIndex = gamePieceForChar(static_cast<char>(incoming));
                if (pieceIndex >= 0)
                {
                    startGameScoped(armedSlot, pieceIndex);
                }
                else
                {
                    Serial.println("   [scope cancelled]");
                }
            }
            pendingSlot   = 0;
            digitsStarted = false;
            continue;
        }

        /* Game mode intercepts its own keys; any other key is dropped so
         * it cannot leak into the normal label/digit handler */
        if (gGameActive)
        {
            switch (incoming)
            {
            case ' ': {
                const BoxPieceId currentPiece = kGamePieces[gGamePieceIndex];
                bool ok;
                if (currentPiece == BoxPieceId::EMPTY)
                {
                    ok = recordCurrentEmptyStep();
                }
                else
                {
                    ok = recordGameSample(gGameSlotIndex, currentPiece, true) ==
                         GameSampleResult::Recorded;
                }
                if (ok)
                {
                    gGameOutlierWarned = false;
                    advanceGameState();
                }
            }
            break;
            case 'a':
                gGameAuto = !gGameAuto;
                Serial.print("   auto-capture ");
                Serial.println(gGameAuto ? "ON" : "OFF");
                displayGameInstruction();
                break;
            case 'r':
                retryGameStep();
                break;
            case 'q':
                stopGameMode();
                break;
            case '?':
                Serial.println("\n   Game mode commands:");
                Serial.println("   SPACE = record, 'a' = toggle auto, 'r' = retry, 'q' = quit");
                displayGameInstruction();
                break;
            default:
                break;
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
        case 'w':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("row reprints work in calibration mode ('c' to switch)");
                break;
            }
            emitGameThresholds();
            break;
        case 'g':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("game mode works in calibration mode ('c' to switch)");
                break;
            }
            if (gGameActive)
            {
                Serial.println("game mode already active ('q' to stop)");
                break;
            }
            if (digitsStarted)
            {
                if (pendingSlot >= 1 && pendingSlot <= kGameSlotCount)
                {
                    gameArmed = true;
                    armedSlot = static_cast<uint8_t>(pendingSlot - 1);
                    Serial.print("   game scoped to slot ");
                    Serial.print(pendingSlot);
                    Serial.println(": piece letter (e f L r l) or 'g' for all");
                }
                else
                {
                    Serial.println("slot out of range for the game (1..12)");
                }
            }
            else
            {
                startGameMode();
            }
            break;
        case 'D':
            dumpPieceClusters();
            break;
        case 'i':
            startClusterImport();
            break;
        case 'R':
            if (gMode != TestMode::Calibration)
            {
                Serial.println("sample reset works in calibration mode ('c' to switch)");
                break;
            }
            if (digitsStarted)
            {
                if (pendingSlot >= 1 && pendingSlot <= kCalibrationSlotCount)
                {
                    resetLabeledSlot(static_cast<uint8_t>(pendingSlot - 1));
                }
                else
                {
                    Serial.println("slot out of range (1..20)");
                }
            }
            else
            {
                Serial.println("type '<slot>R' e.g. '5R' (reset slot 5 samples)");
            }
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

/* USB key shell: the help text, character echo and the command
 * dispatcher with its dfu matcher and digit-prefix state */
#include "dfu_bootloader.h"
#include "hwtest.h"

#ifdef TEST_ADS1115
/* The key list, shown at boot and on demand by '?' */
void printUsbHelp()
{
    Serial.println("   usb: h health, c calibration, s run test, t gaps, x thresholds, ? help");
    Serial.println(
        "   calibration: '<slot><label>' e.g. '5f' (e f b r l L), m = monitor slot, R = reset");
    Serial.println(
        "   game (calibration): 'g' start, SPACE record, 'a' toggle auto, 'r' retry, 'q' quit");
    Serial.println("   clusters: 'D' dump, 'i' import (paste dump, finish with a lone .)");
    Serial.println("   board: type dfu to enter the DFU bootloader (reflash/power-cycle to exit)");
    Serial.println("   echo: typed keys print back, [SP] marks SPACE, import lines show <<<");
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
    /* pendingSlot collects a typed slot number that prefixes a label key
     * ('17L'): digits accumulate in a row and the next label letter ends
     * the slot selection; any other key clears the pending digits. With
     * no explicit slot the label reuses the previously addressed slot */
    static uint16_t pendingSlot    = 0;
    static bool digitsStarted      = false;
    static uint8_t lastLabeledSlot = kCalibrationSlotCount; /* none selected */

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
            continue;
        }

        echoTypedKey(incoming);

        /* While the slot monitor runs, any key is a stop request */
        if (gMonitorActive)
        {
            gMonitorExit  = true;
            pendingSlot   = 0;
            digitsStarted = false;
            /* Drop the rest of the burst: this stop key cuts a command typed
             * while the monitor ran, and the surviving tail would decode as a
             * different command (a bare 'm' falls back to the previously
             * selected slot). The operator retypes it once the monitor has
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
                    ok = recordAllSlotsEmpty();
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
            case 'r': {
                const uint8_t slot     = gGameSlotIndex;
                const BoxPieceId piece = kGamePieces[gGamePieceIndex];
                if (piece == BoxPieceId::EMPTY)
                {
                    /* Reset all slots for EMPTY */
                    for (uint8_t s = 0; s < kGameSlotCount; ++s)
                    {
                        resetLabelPiece(s, BoxPieceId::EMPTY);
                    }
                    Serial.println("   reset all slots EMPTY cluster, retry");
                }
                else
                {
                    resetLabelPiece(slot, piece);
                    gGameSampleIndex = 0;
                    Serial.print("   reset slot ");
                    Serial.print(static_cast<int>(slot) + 1);
                    Serial.print(" ");
                    Serial.print(labelName(static_cast<int>(piece)));
                    Serial.println(" cluster, retry");
                }
                gGameOutlierWarned = false;
                displayGameInstruction();
            }
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

        const char ch    = static_cast<char>(incoming);
        BoxPieceId piece = BoxPieceId::INVALID;
        (void)labelCharToPiece(ch, piece);

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
        case 't':
            runGapAnalysis();
            break;
        case 'x':
            emitThresholdHeader();
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
            startGameMode();
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
            else if (lastLabeledSlot < kCalibrationSlotCount)
            {
                resetLabeledSlot(lastLabeledSlot);
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
            if (digitsStarted)
            {
                if (pendingSlot >= 1 && pendingSlot <= kCalibrationSlotCount)
                {
                    lastLabeledSlot = static_cast<uint8_t>(pendingSlot - 1);
                }
                else
                {
                    Serial.println("slot out of range (1..20)");
                    break;
                }
            }
            else if (lastLabeledSlot >= kCalibrationSlotCount)
            {
                Serial.println("type '<slot>m' e.g. '5m' (monitor slot 5)");
                break;
            }
            runSlotMonitor(lastLabeledSlot);
            break;
        default:
            if (piece == BoxPieceId::INVALID)
            {
                break;
            }
            if (gMode != TestMode::Calibration)
            {
                Serial.println("labels register in calibration mode ('c' to switch)");
                break;
            }
            if (digitsStarted)
            {
                if (pendingSlot >= 1 && pendingSlot <= kCalibrationSlotCount)
                {
                    lastLabeledSlot = static_cast<uint8_t>(pendingSlot - 1);
                }
                else
                {
                    Serial.println("slot out of range (1..20)");
                    break;
                }
            }
            else if (lastLabeledSlot >= kCalibrationSlotCount)
            {
                Serial.println("type '<slot><label>' e.g. '5f' (slot 5 forward)");
                break;
            }
            recordLabelPoint(lastLabeledSlot, piece);
            break;
        }

        digitsStarted = false;
        pendingSlot   = 0;
    }
}
#endif /* TEST_ADS1115 */

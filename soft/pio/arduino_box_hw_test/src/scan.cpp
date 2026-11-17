/* Calibration scan, continuous slot monitor and per-slot raw
 * decoding */
#include "hwtest.h"
#include "i2c3_bitbang.h"

#include <Adafruit_ADS1X15.h>

uint32_t calibration_scan_counter = 0;

const char *scan_label_name(int index)
{
    switch (index)
    {
    case 0:
        return "EMPTY";
    case 1:
        return "FORWARD";
    case 2:
        return "BACKWARD";
    case 3:
        return "TURN_RIGHT";
    case 4:
        return "TURN_LEFT";
    case 5:
        return "LOOP_CALL";
    case 6: /* INVALID: the decode name, never a cluster label */
        return "INVALID";
    default:
        return "?";
    }
}

/* Previous scan pass, kept for the pass-to-pass comparison: every scan
 * stores the mean it read per slot and the next scan prints the signed
 * delta against it. The running min/max on the slot lines span the whole
 * session, so a pass with a piece and one without look like thousands of
 * counts of noise; pairing only the last two passes is what a re-seat or a
 * cover move actually shifts. ads_reset_calibration_stats() zeroes the counter on
 * a mode entry, which also drops this baseline */
static uint16_t scan_prev_mean[kCalibrationSlotCount];
static bool scan_prev_valid[kCalibrationSlotCount];
static bool scan_has_prev = false;

/* Delta past which the comparison flags a slot: the readings gigue is 0-16
 * counts on I2C3 and around 100 on I2C1, so this catches real movement (a
 * re-seated piece, a cover shift) without flagging the bus noise */
static constexpr uint32_t kScanDeltaAlert = 150;

/* Continuous slot monitor ('m'): while active, usb keys only stop it */
bool monitor_active = false;
bool monitor_exit   = false;

/* Frozen slot monitor ('x'): only the SPACE-opened windows sample, so an
 * orientation sweep logs one clean line per pose, unpolluted by the
 * rotations happening between windows */
bool freeze_active    = false;
bool freeze_measure   = false;
bool freeze_measuring = false;
bool freeze_exit      = false;

/* Length of one frozen-monitor measurement window */
constexpr uint32_t kFreezeWindowMs = 3000;

/* Defined below, next to the scan that also decodes with it */
static const char *scan_piece_label_for_raw(uint8_t slotIndex, int16_t raw);

/* Continuous readout of one slot: ~5 mean reads per second with running
 * min/max and the live decoded piece from the slot's compiled row. Any usb
 * key stops it. Clamp detection: a max pinned at the top of the scale (near
 * 26400 on the 3V3 rail) means the hall sensor output is saturated, not
 * that the piece reads high */
void scan_run_slot_monitor(uint8_t slotIndex)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    Serial.print("--- MONITOR slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" (");
    Serial.print(entry.busName);
    Serial.println("), any key to stop ---");
    uint16_t minRaw = 0xFFFF;
    uint16_t maxRaw = 0;
    uint32_t count  = 0;
    uint64_t sum    = 0;

    monitor_active = true;
    monitor_exit   = false;
    while (!monitor_exit)
    {
        int16_t rawMean = 0;
        bool readOk     = ads_read_single_ended_mean(&entry, channel, rawMean);
        if (!readOk)
        {
            Serial.println("   read TIMEOUT/FAIL, recovering module");
            ads_recover_module(&entry);
            readOk = ads_read_single_ended_mean(&entry, channel, rawMean);
        }
        if (readOk)
        {
            const uint16_t raw = static_cast<uint16_t>(rawMean);
            if (raw < minRaw)
            {
                minRaw = raw;
            }
            if (raw > maxRaw)
            {
                maxRaw = raw;
            }
            sum += raw;
            ++count;
        }

        /* Group the readouts by four so the addressed ADS module is
         * always visible: one group header per four readings */
        if (readOk && count % 4 == 1)
        {
            Serial.print("   [ADS module ");
            Serial.print(static_cast<int>(slotIndex / 4) + 1);
            Serial.print("/5: ");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.print(", slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.print(" = channel ");
            Serial.print(static_cast<int>(channel) + 1);
            Serial.println("/4]");
        }

        Serial.print("   slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        Serial.print(": raw=");
        if (readOk)
        {
            Serial.print(rawMean);
        }
        else
        {
            Serial.print("----");
        }
        Serial.print(" min=");
        if (count > 0)
        {
            Serial.print(static_cast<int>(minRaw));
            Serial.print(" max=");
            Serial.print(static_cast<int>(maxRaw));
            if (readOk)
            {
                /* What this slot's compiled row sees right now: the same
                 * decode the scan uses, so a placement can be checked live */
                Serial.print("  ");
                Serial.print(scan_piece_label_for_raw(slotIndex, rawMean));
            }
            Serial.println();
        }
        else
        {
            Serial.println("---- max=----");
        }

        /* The parser only sets monitor_exit here, so the sampling
         * period doubles as the usb response time */
        shell_delay_with_usb(200);
    }
    monitor_active = false;

    Serial.println("--- MONITOR stopped ---");
    if (count > 0)
    {
        Serial.print("   samples n=");
        Serial.print(static_cast<int>(count));
        Serial.print(" mean=");
        Serial.print(static_cast<int>(sum / count));
        Serial.print(" min=");
        Serial.print(static_cast<int>(minRaw));
        Serial.print(" max=");
        Serial.print(static_cast<int>(maxRaw));
        Serial.print(" span=");
        Serial.print(static_cast<int>(maxRaw - minRaw));
        Serial.print("  ");
        Serial.println(scan_piece_label_for_raw(slotIndex, static_cast<int16_t>(sum / count)));
    }
}

/* Frozen slot monitor ('x'): waits without sampling; a SPACE opens a fixed
 * measurement window whose stats print as one self-contained line, so
 * rotating or placing the piece between two windows cannot pollute any
 * stat. The quit key stops a window early, the partial line still prints */
void scan_run_frozen_slot_monitor(uint8_t slotIndex)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    Serial.print("--- FROZEN MONITOR slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" (");
    Serial.print(entry.busName);
    Serial.println("), SPACE = measure, any other key to quit ---");
    freeze_active    = true;
    freeze_measure   = false;
    freeze_measuring = false;
    freeze_exit      = false;
    while (!freeze_exit)
    {
        shell_delay_with_usb(50);
        if (!freeze_measure)
        {
            continue;
        }
        freeze_measure   = false;
        freeze_measuring = true;

        uint16_t minRaw        = 0xFFFF;
        uint16_t maxRaw        = 0;
        uint32_t count         = 0;
        uint64_t sum           = 0;
        const uint32_t startMs = millis();
        while (millis() - startMs < kFreezeWindowMs && !freeze_exit)
        {
            int16_t rawMean = 0;
            bool readOk     = ads_read_single_ended_mean(&entry, channel, rawMean);
            if (!readOk)
            {
                ads_recover_module(&entry);
                readOk = ads_read_single_ended_mean(&entry, channel, rawMean);
            }
            if (readOk)
            {
                const uint16_t raw = static_cast<uint16_t>(rawMean);
                if (raw < minRaw)
                {
                    minRaw = raw;
                }
                if (raw > maxRaw)
                {
                    maxRaw = raw;
                }
                sum += raw;
                ++count;
            }
            shell_delay_with_usb(20);
        }
        freeze_measuring = false;

        Serial.print("   slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        if (count == 0)
        {
            Serial.println(": no valid reading in window (ADS timeout)");
            continue;
        }
        const uint16_t mean = static_cast<uint16_t>(sum / count);
        Serial.print(": n=");
        Serial.print(static_cast<int>(count));
        Serial.print(" mean=");
        Serial.print(mean);
        Serial.print(" min=");
        Serial.print(static_cast<int>(minRaw));
        Serial.print(" max=");
        Serial.print(static_cast<int>(maxRaw));
        Serial.print(" span=");
        Serial.print(static_cast<int>(maxRaw - minRaw));
        Serial.print("  ");
        Serial.println(scan_piece_label_for_raw(slotIndex, static_cast<int16_t>(mean)));
    }
    freeze_active = false;
    Serial.println("--- FROZEN MONITOR stopped ---");
}

/* Decode one raw reading with this slot's row of the per-slot threshold
 * table. The columns are ascending piece starts, so the first column below
 * which nothing matches means the slot is not calibrated: it decodes
 * INVALID rather than guessing with the global fallbacks. This is the only
 * decode in the firmware: the monitors, the scan and the radio send all go
 * through it, so a label printed here and a piece sent over the air can
 * never come from two different walks */
/* Which piece each band carries. The bands ascend with the raw reading, so
 * this array is the single place where a name is bound to a physical family.
 * This batch was mounted with the opposite pole throughout, which puts the
 * greens at the bottom and the blues at the top; the operator asked for the
 * greens to be FORWARD and the reds LOOP_CALL, so the mapping follows the
 * measurement instead of asking twenty magnets to be flipped. Rebuilding the
 * pieces the other way round flips this array, nothing else */
static const BoxPieceId kBandPieces[5] = {
    BoxPieceId::FORWARD,    /* band 0: green 4 mm, the most negative */
    BoxPieceId::TURN_RIGHT, /* band 1: yellow 2 mm */
    BoxPieceId::EMPTY,      /* band 2: the slot's own idle level */
    BoxPieceId::LOOP_CALL,  /* band 3: red 2 mm */
    BoxPieceId::TURN_LEFT,  /* band 4: blue 4 mm, the most positive */
};

BoxPieceId scan_piece_for_raw(uint8_t slotIndex, int16_t raw)
{
    if (raw < 0)
    {
        return BoxPieceId::INVALID;
    }
    const uint16_t *t = GASPETTO_ADC_SLOT_THRESHOLDS[slotIndex];
    if (t[GASPETTO_ADC_SLOT_COL_TURN_LEFT] == 0 || raw < t[GASPETTO_ADC_SLOT_COL_TURN_LEFT])
    {
        return BoxPieceId::INVALID;
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_LOOP_START])
    {
        return kBandPieces[0];
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_EMPTY])
    {
        return kBandPieces[1];
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_TURN_RIGHT])
    {
        return kBandPieces[2];
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_FORWARD])
    {
        return kBandPieces[3];
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_TO_REMOVE])
    {
        return kBandPieces[4];
    }
    return BoxPieceId::INVALID;
}

/* What the scan prints: the enum above under its label */
static const char *scan_piece_label_for_raw(uint8_t slotIndex, int16_t raw)
{
    return scan_label_name(static_cast<int>(scan_piece_for_raw(slotIndex, raw)));
}

void scan_run_piece_calibration()
{
    /* This pass's mean per slot, filled as the loop goes, and whether the
     * slot produced a reading: the comparison block below pairs them with
     * the previous pass, and a failed read keeps the old entry so a
     * transient timeout is not reported as a movement */
    uint16_t passMean[kCalibrationSlotCount] = {};
    bool passValid[kCalibrationSlotCount]    = {};

    /* ads_reset_calibration_stats() zeroes the counter on a mode entry, so a zero
     * here means a fresh session and the stored pass belongs to before it */
    if (calibration_scan_counter == 0)
    {
        for (uint8_t slotIndex = 0; slotIndex < kCalibrationSlotCount; ++slotIndex)
        {
            scan_prev_valid[slotIndex] = false;
        }
        scan_has_prev = false;
    }

    Serial.println("\n--- PIECE THRESHOLD CALIBRATION ---");
    Serial.print("scan=");
    Serial.println(++calibration_scan_counter);
    Serial.print("mean samples=");
    Serial.print(kAdsMeanSamples);
    /* Decoding compares against each slot's own row, printed on every slot
     * line below; the global macros this header used to list are the
     * production legacy table and the scan never used them */
    Serial.println("decode: per-slot rows, see row=[...] on each slot line");

    /* No rail held around the scan: the readings below pulse it themselves,
     * exactly like the game and 'p' do. Holding it here would warm the board
     * through the whole scan, so this path would calibrate against a
     * temperature those two never read - which is what the offsets were.
     * Only the probe needs it, and that one takes it inside the loop */
    for (uint8_t deviceIndex = 0; deviceIndex < kAdsCount; ++deviceIndex)
    {
        const AdsTestEntry &entry = kAdsTests[deviceIndex];

        /* Check if device is accessible. The chip hangs on the sensor rail,
         * so take it for the probe and give it straight back: the readings
         * that follow must pulse it the way the game and 'p' do */
        const bool probeRail = power_sensor_rail_acquire();

        bool deviceFound;
        if (entry.useBitBang)
        {
            deviceFound = i2c3_bb_probe(entry.address);
        }
        else
        {
            Adafruit_ADS1115 ads;
            deviceFound = ads.begin(entry.address, entry.wire);
        }

        power_sensor_rail_release(probeRail);

        if (!deviceFound)
        {
            Serial.print("   [MISS] ");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.println(" calibration scan skipped");
            continue;
        }

        for (uint8_t ch = 0; ch < 4; ++ch)
        {
            int16_t rawMean = 0;
            bool readOk     = ads_read_single_ended_mean(&entry, ch, rawMean);

            if (!readOk && ads_is_i2c3_entry(&entry))
            {
                ads_recover_i2c3_bus();
                readOk = ads_read_single_ended_mean(&entry, ch, rawMean);
            }

            const uint8_t slotIndex = static_cast<uint8_t>(deviceIndex * 4 + ch);
            Serial.print("   slot ");
            Serial.print(slotIndex + 1);
            Serial.print(" [");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.print(" ch");
            Serial.print(ch);
            Serial.print("] ");

            /* The slot's own decode boundaries, the exact numbers
             * scan_piece_label_for_raw just compared against; uncalibrated slots
             * decode INVALID whatever they read */
            const uint16_t *row = GASPETTO_ADC_SLOT_THRESHOLDS[slotIndex];
            if (row[GASPETTO_ADC_SLOT_COL_TURN_LEFT] == 0U)
            {
                Serial.print("row=uncalibrated ");
            }
            else
            {
                Serial.print("row=[");
                for (uint8_t c = 0; c < GASPETTO_ADC_COLS; ++c)
                {
                    if (c > 0)
                    {
                        Serial.print(' ');
                    }
                    Serial.print(row[c]);
                }
                Serial.print("] ");
            }

            if (!readOk)
            {
                Serial.println("TIMEOUT/FAIL");
                continue;
            }

            ads_update_calibration_stats(slotIndex, rawMean);
            passMean[slotIndex]           = static_cast<uint16_t>(rawMean);
            passValid[slotIndex]          = true;
            const CalibrationStats &stats = calibration_stats[slotIndex];

            Serial.print("mean=");
            Serial.print(rawMean);
            Serial.print(" min=");
            Serial.print(stats.minRaw);
            Serial.print(" max=");
            Serial.print(stats.maxRaw);
            Serial.print(" span=");
            Serial.print(stats.maxRaw - stats.minRaw);
            Serial.print(" => ");
            Serial.println(scan_piece_label_for_raw(slotIndex, rawMean));
        }
    }

    /* Pass-to-pass comparison: the running min/max on the lines above span
     * the whole session, so a pass with a piece and one without read as
     * thousands of counts of noise. Pairing only the last two passes shows
     * what actually moved - a re-seated piece, a shifted cover - and a delta
     * past kScanDeltaAlert is flagged */
    Serial.print("--- PASS COMPARISON");
    if (scan_has_prev)
    {
        Serial.print(" vs scan ");
        Serial.print(calibration_scan_counter - 1);
    }
    Serial.println(" ---");

    if (!scan_has_prev)
    {
        Serial.println("   first pass of this session, baseline stored: rerun to compare");
    }
    else
    {
        for (uint8_t slotIndex = 0; slotIndex < kCalibrationSlotCount; ++slotIndex)
        {
            Serial.print("   slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.print(": ");

            if (!passValid[slotIndex])
            {
                Serial.println("read FAIL, no comparison");
                continue;
            }
            if (!scan_prev_valid[slotIndex])
            {
                Serial.print(passMean[slotIndex]);
                Serial.println(" (no previous pass)");
                continue;
            }

            const int32_t delta = static_cast<int32_t>(passMean[slotIndex]) -
                                  static_cast<int32_t>(scan_prev_mean[slotIndex]);
            Serial.print(scan_prev_mean[slotIndex]);
            Serial.print(" -> ");
            Serial.print(passMean[slotIndex]);
            Serial.print("  delta=");
            if (delta >= 0)
            {
                Serial.print('+');
            }
            Serial.print(delta);
            if ((delta > static_cast<int32_t>(kScanDeltaAlert)) ||
                (delta < -static_cast<int32_t>(kScanDeltaAlert)))
            {
                Serial.print("  <<");
            }
            Serial.println();
        }
    }

    /* Hand this pass to the next comparison. Mean and validity are copied
     * per slot: a slot that failed keeps its last valid reading as the
     * reference instead of losing the baseline */
    for (uint8_t slotIndex = 0; slotIndex < kCalibrationSlotCount; ++slotIndex)
    {
        if (passValid[slotIndex])
        {
            scan_prev_mean[slotIndex]  = passMean[slotIndex];
            scan_prev_valid[slotIndex] = true;
        }
    }
    scan_has_prev = true;
}

/* ========================================== */
/* Scan -> send helpers, used by the "p" key  */
/* ========================================== */

/* One slot of the scan: read, decode, report. Returns false when the slot
 * decodes INVALID - a failed read, an uncalibrated row or a value outside
 * every band - which aborts the run: an INVALID is a fault, not the place
 * where the sequence stops */
static bool scan_one_slot(uint8_t slot, BoxBoardPieces &board)
{
    const AdsTestEntry &entry = kAdsTests[slot / 4];
    const uint8_t channel     = slot % 4;
    int16_t rawMean           = 0;

    bool readOk = ads_read_single_ended_mean(&entry, channel, rawMean);
    if (!readOk)
    {
        ads_recover_module(&entry);
        readOk = ads_read_single_ended_mean(&entry, channel, rawMean);
    }

    Serial.print("   slot ");
    Serial.print(static_cast<int>(slot) + 1);
    if (!readOk)
    {
        Serial.println(": READ FAIL -> INVALID");
        return false;
    }

    const BoxPieceId piece = scan_piece_for_raw(slot, rawMean);
    Serial.print(": raw=");
    Serial.print(rawMean);
    Serial.print("  ");
    Serial.println(scan_label_name(static_cast<int>(piece)));
    if (piece == BoxPieceId::INVALID)
    {
        return false;
    }

    board[slot] = piece;
    return true;
}

/* Scan a zone in slot order and stop at the first EMPTY: the pieces are a
 * contiguous run starting at slot 1, so an empty slot means the sequence is
 * over and whatever lies after it is out of the program - what was already
 * read stays. Returns false on the first INVALID so the caller can raise
 * the build-error LEDs */
bool scan_zone(uint8_t firstSlot, uint8_t endSlot, BoxBoardPieces &board, bool &sawLoopCall)
{
    for (uint8_t slot = firstSlot; slot < endSlot; ++slot)
    {
        if (!scan_one_slot(slot, board))
        {
            return false;
        }
        if (board[slot] == BoxPieceId::EMPTY)
        {
            Serial.print("[send] slot ");
            Serial.print(static_cast<int>(slot) + 1);
            Serial.println(" empty: sequence ends here");
            return true;
        }
        if (board[slot] == BoxPieceId::LOOP_CALL)
        {
            sawLoopCall = true;
        }
    }
    return true;
}

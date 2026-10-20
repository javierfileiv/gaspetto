/* Calibration scan, continuous slot monitor and per-slot raw
 * decoding */
#include "hwtest.h"
#include "i2c3_bitbang.h"

#include <Adafruit_ADS1X15.h>

uint32_t gCalibrationScanCounter = 0;

/* Continuous slot monitor ('m'): while active, usb keys only stop it */
bool gMonitorActive = false;
bool gMonitorExit   = false;

/* Defined below, next to the scan that also decodes with it */
static const char *pieceLabelForRaw(uint8_t slotIndex, int16_t raw);

/* Continuous readout of one slot: ~5 mean reads per second with running
 * min/max and the live decoded piece from the slot's compiled row. Any usb
 * key stops it. Clamp detection: a max pinned at the top of the scale (near
 * 26400 on the 3V3 rail) means the hall sensor output is saturated, not
 * that the piece reads high */
void runSlotMonitor(uint8_t slotIndex)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    Serial.print("--- MONITOR slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" (");
    Serial.print(entry.busName);
    Serial.println("), any key to stop ---");
    if (!gCalibrationRailsOn)
    {
        Serial.println("   NOTE: 3V3 rail is OFF, readings will be garbage");
    }

    uint16_t minRaw = 0xFFFF;
    uint16_t maxRaw = 0;
    uint32_t count  = 0;
    uint64_t sum    = 0;

    gMonitorActive = true;
    gMonitorExit   = false;
    while (!gMonitorExit)
    {
        int16_t rawMean = 0;
        bool readOk     = readAdsSingleEndedMean(&entry, channel, rawMean);
        if (!readOk)
        {
            Serial.println("   read TIMEOUT/FAIL, recovering module");
            recoverAdsModule(&entry);
            readOk = readAdsSingleEndedMean(&entry, channel, rawMean);
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
                Serial.print(pieceLabelForRaw(slotIndex, rawMean));
            }
            Serial.println();
        }
        else
        {
            Serial.println("---- max=----");
        }

        /* The parser only sets gMonitorExit here, so the sampling
         * period doubles as the usb response time */
        delayWithUsb(200);
    }
    gMonitorActive = false;

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
        Serial.println(pieceLabelForRaw(slotIndex, static_cast<int16_t>(sum / count)));
    }
}

/* Decode one raw reading with this slot's row of the per-slot threshold
 * table. The columns are ascending piece starts, so the first column below
 * which nothing matches means the slot is not calibrated: it decodes
 * INVALID rather than guessing with the global fallbacks */
static const char *pieceLabelForRaw(uint8_t slotIndex, int16_t raw)
{
    if (raw < 0)
    {
        return "INVALID";
    }
    const uint16_t *t = GASPETTO_ADC_SLOT_THRESHOLDS[slotIndex];
    if (t[GASPETTO_ADC_SLOT_COL_TURN_LEFT] == 0 || raw < t[GASPETTO_ADC_SLOT_COL_TURN_LEFT])
    {
        return "INVALID";
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_BACKWARD])
    {
        return "TURN_LEFT";
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_EMPTY])
    {
        /* Band 1 is the loop piece now; the shared enum column name still
         * says BACKWARD until the production migration renames it */
        return "LOOP_CALL";
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_TURN_RIGHT])
    {
        return "EMPTY";
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_FORWARD])
    {
        return "TURN_RIGHT";
    }
    if (raw < t[GASPETTO_ADC_SLOT_COL_LOOP_START])
    {
        return "FORWARD";
    }
    return "INVALID";
}

void runPieceCalibrationScan()
{
    Serial.println("\n--- PIECE THRESHOLD CALIBRATION ---");
    Serial.print("scan=");
    Serial.println(++gCalibrationScanCounter);
    Serial.print("mean samples=");
    Serial.print(kAdsMeanSamples);
    /* Decoding compares against each slot's own row, printed on every slot
     * line below; the global macros this header used to list are the
     * production legacy table and the scan never used them */
    Serial.println("decode: per-slot rows, see row=[...] on each slot line");

    for (uint8_t deviceIndex = 0; deviceIndex < kAdsCount; ++deviceIndex)
    {
        const AdsTestEntry &entry = kAdsTests[deviceIndex];

        // Check if device is accessible
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
            bool readOk     = readAdsSingleEndedMean(&entry, ch, rawMean);

            if (!readOk && isI2c3Entry(&entry))
            {
                recoverI2c3Bus();
                readOk = readAdsSingleEndedMean(&entry, ch, rawMean);
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
             * pieceLabelForRaw just compared against; uncalibrated slots
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

            updateCalibrationStats(slotIndex, rawMean);
            const CalibrationStats &stats = gCalibrationStats[slotIndex];

            Serial.print("mean=");
            Serial.print(rawMean);
            Serial.print(" min=");
            Serial.print(stats.minRaw);
            Serial.print(" max=");
            Serial.print(stats.maxRaw);
            Serial.print(" span=");
            Serial.print(stats.maxRaw - stats.minRaw);
            Serial.print(" => ");
            Serial.println(pieceLabelForRaw(slotIndex, rawMean));
        }
    }

    Serial.println("--- LABELED PIECE SAMPLES ---");
    for (uint8_t slotIndex = 0; slotIndex < kCalibrationSlotCount; ++slotIndex)
    {
        for (int label = 0; label < 6; ++label)
        {
            const LabelCluster &cluster = gPieceClusters[slotIndex][label];
            if (cluster.count == 0)
            {
                continue;
            }
            Serial.print("   slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.print(" ");
            Serial.print(labelName(label));
            Serial.print(": n=");
            Serial.print(cluster.count);
            Serial.print(" raw ");
            Serial.print(cluster.min);
            Serial.print("-");
            Serial.print(cluster.max);
            Serial.print(" mean=");
            Serial.println(static_cast<int>(cluster.sum / cluster.count));
        }
    }
}

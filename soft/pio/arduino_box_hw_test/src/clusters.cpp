/* Piece label clusters: the per (slot, piece) accumulators shared by
 * the guided game walk, the manual labels, the text dump (D) and the
 * restore import (i) */
#include "hwtest.h"

/* Cluster restore mode: while active every serial character goes to the
 * cluster parser instead of the key handler, until a lone "." line ends it.
 * A reflashed board loses gPieceClusters (RAM only), so this is how a
 * finished campaign survives a reflash */
bool gClusterImportActive         = false;
static char gClusterImportBuf[48] = {};
static uint8_t gClusterImportLen  = 0;
static uint16_t gClusterImported  = 0;

LabelCluster gPieceClusters[kCalibrationSlotCount][6] = {};

const char *labelName(int index)
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
    default:
        return "?";
    }
}

void addLabelPoint(uint8_t slotIndex, BoxPieceId piece, uint16_t raw)
{
    LabelCluster &cluster = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];
    if (cluster.count == 0)
    {
        cluster.min   = raw;
        cluster.max   = raw;
        cluster.sum   = raw;
        cluster.count = 1;
        return;
    }
    if (raw < cluster.min)
    {
        cluster.min = raw;
    }
    if (raw > cluster.max)
    {
        cluster.max = raw;
    }
    cluster.sum += raw;
    ++cluster.count;
}

void resetLabeledSlot(uint8_t slotIndex)
{
    for (int label = 0; label < 6; ++label)
    {
        gPieceClusters[slotIndex][label] = LabelCluster{};
    }
    Serial.print("   slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.println(" samples reset");
}

void resetLabelPiece(uint8_t slotIndex, BoxPieceId piece)
{
    gPieceClusters[slotIndex][static_cast<uint8_t>(piece)] = LabelCluster{};
}

/* Serial dump of every cluster, one line per (slot, label), so a campaign
 * survives a reflash: gPieceClusters lives in RAM only. Line format is
 * "<slot 1-based> <label index> <min> <max> <count> <sum>" */
void dumpPieceClusters()
{
    Serial.println("--- CLUSTER DUMP (slot label min max count sum) ---");
    uint16_t dumped = 0;
    for (uint8_t slot = 0; slot < kCalibrationSlotCount; ++slot)
    {
        for (int label = 0; label < 6; ++label)
        {
            const LabelCluster &cl = gPieceClusters[slot][label];
            if (cl.count == 0)
            {
                continue;
            }
            Serial.print(static_cast<int>(slot) + 1);
            Serial.print(' ');
            Serial.print(label);
            Serial.print(' ');
            Serial.print(cl.min);
            Serial.print(' ');
            Serial.print(cl.max);
            Serial.print(' ');
            Serial.print(cl.count);
            Serial.print(' ');
            Serial.println(cl.sum);
            ++dumped;
        }
    }
    Serial.print("   clusters: ");
    Serial.println(dumped);
    Serial.println("--- END CLUSTER DUMP ---");
}

/* Enter restore mode: the whole set is cleared first, so a pasted dump
 * replaces the data instead of merging into stale clusters */
void startClusterImport()
{
    for (uint8_t slot = 0; slot < kCalibrationSlotCount; ++slot)
    {
        for (int label = 0; label < 6; ++label)
        {
            gPieceClusters[slot][label] = LabelCluster{};
        }
    }
    gClusterImportActive = true;
    gClusterImportLen    = 0;
    gClusterImported     = 0;
    Serial.println("--- CLUSTER IMPORT: paste the dump, then a line with only . ---");
    Serial.println("   (existing clusters were cleared)");
}

/* Read one decimal token, leaving p on the character after it */
static bool clusterToken(const char *&p, uint32_t &out)
{
    while (*p == ' ')
    {
        ++p;
    }
    if (*p < '0' || *p > '9')
    {
        return false;
    }
    uint32_t value = 0;
    while (*p >= '0' && *p <= '9')
    {
        value = value * 10U + static_cast<uint32_t>(*p - '0');
        ++p;
    }
    out = value;
    return true;
}

/* Apply one dump line back into the cluster table */
static bool applyClusterLine(const char *line)
{
    const char *p = line;
    uint32_t v[6] = {};
    for (uint8_t i = 0; i < 6; ++i)
    {
        if (!clusterToken(p, v[i]))
        {
            return false;
        }
    }
    while (*p == ' ')
    {
        ++p;
    }
    if (*p != '\0')
    {
        return false; /* trailing garbage */
    }
    if (v[0] < 1 || v[0] > kCalibrationSlotCount || v[1] >= 6 || v[4] == 0)
    {
        return false;
    }
    if (v[2] > 0xFFFFU || v[3] > 0xFFFFU || v[4] > 0xFFFFU || v[3] < v[2])
    {
        return false;
    }
    LabelCluster &cl = gPieceClusters[v[0] - 1][v[1]];
    cl.min           = static_cast<uint16_t>(v[2]);
    cl.max           = static_cast<uint16_t>(v[3]);
    cl.count         = static_cast<uint16_t>(v[4]);
    cl.sum           = v[5];
    return true;
}

/* Restore mode sink: accumulate raw characters into lines terminated by CR
 * or LF (a Windows paste sends CR alone and lines would never complete),
 * apply each one and stop on a line holding only "." */
void handleClusterImportChar(char c)
{
    if (c == '\r' || c == '\n')
    {
        gClusterImportBuf[gClusterImportLen] = '\0';
        gClusterImportLen                    = 0;
        if (gClusterImportBuf[0] == '.' && gClusterImportBuf[1] == '\0')
        {
            gClusterImportActive = false;
            Serial.println("<<< .");
            Serial.print("--- CLUSTER IMPORT DONE, restored ");
            Serial.print(gClusterImported);
            Serial.println(" ---");
            Serial.println("   'D' re-dumps to verify");
            return;
        }
        if (gClusterImportBuf[0] == '\0')
        {
            return;
        }
        if (applyClusterLine(gClusterImportBuf))
        {
            ++gClusterImported;
            /* Per-line echo of the accepted paste: progress without drowning
             * the screen in characters; rejected lines already print
             * themselves in the [IMPORT] skipped report */
            Serial.print("<<< ");
            Serial.println(gClusterImportBuf);
        }
        else
        {
            Serial.print("   [IMPORT] skipped: ");
            Serial.println(gClusterImportBuf);
        }
        return;
    }
    if (static_cast<size_t>(gClusterImportLen) + 1U >= sizeof(gClusterImportBuf))
    {
        gClusterImportLen = 0; /* line too long: drop it */
        return;
    }
    gClusterImportBuf[gClusterImportLen++] = c;
}

// ==========================================

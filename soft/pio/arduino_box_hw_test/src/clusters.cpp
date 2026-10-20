/* Piece label clusters: the per (slot, piece) accumulators shared by
 * the guided game walk and the monitor decode */
#include "hwtest.h"

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

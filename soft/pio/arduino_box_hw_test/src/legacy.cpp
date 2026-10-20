/* Legacy global-threshold tools: manual <slot><label> points, the t
 * gap analysis and the x header emitter. They drive the shared #define
 * table production still decodes with; they leave with the migration */
#include "hwtest.h"

void recordLabelPoint(uint8_t slotIndex, BoxPieceId piece)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    int16_t rawMean = 0;
    bool readOk     = readAdsSingleEndedMean(&entry, channel, rawMean);
    if (!readOk)
    {
        recoverAdsModule(&entry);
        readOk = readAdsSingleEndedMean(&entry, channel, rawMean);
    }
    if (!readOk)
    {
        Serial.print("   slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        Serial.println(" read TIMEOUT/FAIL, point not recorded");
        return;
    }

    const uint16_t raw           = static_cast<uint16_t>(rawMean);
    const LabelCluster &existing = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];

    /* Outlier guard: reject samples far outside the already labeled range,
     * so a bad contact or a misplaced piece cannot silently stretch the
     * cluster. Reset the slot cluster ('R') to re-record from scratch */
    if (existing.count > 0)
    {
        const uint32_t spanBottom =
            existing.min > existing.min / 5 ? existing.min - existing.min / 5 : 0;
        const uint32_t spanTop = existing.max + existing.max / 5;
        if (raw < spanBottom || raw > spanTop)
        {
            Serial.print("   [OUTLIER IGNORED] slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.print(" ");
            Serial.print(labelName(static_cast<int>(piece)));
            Serial.print(" raw=");
            Serial.print(raw);
            Serial.print(" expected.raw ");
            Serial.print(static_cast<int>(spanBottom));
            Serial.print("-");
            Serial.println(static_cast<int>(spanTop));
            return;
        }
    }

    addLabelPoint(slotIndex, piece, raw);
    const LabelCluster &cluster = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];

    Serial.print("   slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" ");
    Serial.print(labelName(static_cast<int>(piece)));
    Serial.print(": raw=");
    Serial.print(raw);
    Serial.print(" n=");
    Serial.println(cluster.count);
}
/* One labeled piece type seen across all slots, min/max in raw counts */
struct LabelRange
{
    int label;
    uint32_t min;
    uint32_t max;
    uint32_t count;
};

/* Threshold macro name for a piece type, nullptr when it has none */
static const char *thresholdStartName(int label)
{
    switch (label)
    {
    case 0:
        return "GASPETTO_ADC_THRESHOLD_EMPTY_START";
    case 1:
        return "GASPETTO_ADC_THRESHOLD_FORWARD_START";
    case 2:
        return "GASPETTO_ADC_THRESHOLD_BACKWARD_START";
    case 3:
        return "GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START";
    case 4:
        return "GASPETTO_ADC_THRESHOLD_TURN_LEFT_START";
    case 5:
        return "GASPETTO_ADC_THRESHOLD_LOOP_START";
    default:
        return nullptr;
    }
}

/* Aggregate every labeled cluster by piece type, lowest first */
static void collectSortedPieceRanges(LabelRange *ranges, uint8_t &written)
{
    written = 0;
    for (int label = 0; label < 6; ++label)
    {
        uint32_t min   = 0xFFFFFFFFUL;
        uint32_t max   = 0;
        uint32_t count = 0;
        for (uint8_t slotIndex = 0; slotIndex < kCalibrationSlotCount; ++slotIndex)
        {
            const LabelCluster &cluster = gPieceClusters[slotIndex][label];
            if (cluster.count == 0)
            {
                continue;
            }
            count += cluster.count;
            if (cluster.min < min)
            {
                min = cluster.min;
            }
            if (cluster.max > max)
            {
                max = cluster.max;
            }
        }
        if (count > 0)
        {
            ranges[written].label = label;
            ranges[written].min   = min;
            ranges[written].max   = max;
            ranges[written].count = count;
            ++written;
        }
    }

    /* lowest first */
    for (uint8_t i = 0; i < written; ++i)
    {
        for (uint8_t j = i + 1; j < written; ++j)
        {
            if (ranges[j].min < ranges[i].min)
            {
                const LabelRange swap = ranges[i];
                ranges[i]             = ranges[j];
                ranges[j]             = swap;
            }
        }
    }
}

void runGapAnalysis()
{
    LabelRange ranges[6];
    uint8_t written = 0;
    collectSortedPieceRanges(ranges, written);
    Serial.println("--- PIECE THRESHOLD ANALYSIS ---");
    if (written == 0)
    {
        Serial.println("   no labeled samples; enter calibration ('c') then '<slot><label>'");
        return;
    }
    for (uint8_t i = 0; i < written; ++i)
    {
        Serial.print("   ");
        Serial.print(labelName(ranges[i].label));
        Serial.print(": raw ");
        Serial.print(static_cast<int>(ranges[i].min));
        Serial.print("-");
        Serial.print(static_cast<int>(ranges[i].max));
        Serial.print(", n=");
        Serial.print(static_cast<int>(ranges[i].count));
        Serial.println();
    }

    for (uint8_t i = 0; i + 1 < written; ++i)
    {
        const LabelRange &lower = ranges[i];
        const LabelRange &upper = ranges[i + 1];
        if (lower.max >= upper.min)
        {
            Serial.print("   [AMBIGUOUS] ");
            Serial.print(labelName(lower.label));
            Serial.print(" and ");
            Serial.print(labelName(upper.label));
            Serial.println(" ranges overlap; separate them before trusting thresholds");
            continue;
        }

        const uint32_t suggested = (lower.max + upper.min) / 2;
        Serial.print("   #define ");
        Serial.print(thresholdStartName(upper.label));
        Serial.print(" ");
        Serial.println(static_cast<int>(suggested));
    }
    Serial.println("   note: top of highest labeled piece informs LOOP_END");
}

void emitThresholdHeader()
{
    LabelRange ranges[6];
    uint8_t written = 0;
    collectSortedPieceRanges(ranges, written);

    Serial.println("--- GENERATED piece_thresholds.h BLOCK (raw ADS1115 counts) ---");
    if (written == 0)
    {
        Serial.println("   no labeled samples; enter calibration ('c') then '<slot><label>'");
        return;
    }

    Serial.println("#pragma once");
    Serial.println();
    Serial.println("/* Generated by arduino_box_hw_test from labeled piece samples */");

    /* Neighborhoods drive the defines: the gap between two adjacent
     * labeled piece types becomes the upper piece's *_START */
    for (uint8_t i = 0; i + 1 < written; ++i)
    {
        const LabelRange &lower = ranges[i];
        const LabelRange &upper = ranges[i + 1];
        const char *name        = thresholdStartName(upper.label);
        if (lower.max >= upper.min)
        {
            Serial.print("/* AMBIGUOUS: cannot derive ");
            Serial.print(name);
            Serial.print(" (");
            Serial.print(labelName(lower.label));
            Serial.print(" overlaps ");
            Serial.print(labelName(upper.label));
            Serial.println(") */");
            continue;
        }

        const uint32_t suggested = (lower.max + upper.min) / 2;
        Serial.print("#define ");
        Serial.print(name);
        Serial.print(" ");
        Serial.print(static_cast<int>(suggested));
        Serial.print("  /* gap raw ");
        Serial.print(static_cast<int>(lower.max));
        Serial.print("-");
        Serial.print(static_cast<int>(upper.min));
        Serial.print(", n=");
        Serial.print(static_cast<int>(lower.count + upper.count));
        Serial.println(" */");
    }

    /* LOOP_END just above the top of the highest labeled piece */
    const LabelRange &top = ranges[written - 1];
    Serial.print("#define GASPETTO_ADC_THRESHOLD_LOOP_END ");
    Serial.print(static_cast<int>(top.max + 1));
    Serial.print("  /* above ");
    Serial.print(labelName(top.label));
    Serial.print(" top raw ");
    Serial.print(static_cast<int>(top.max));
    Serial.println(" */");
    Serial.println("#define GASPETTO_ADC_THRESHOLD_MAX 32767");
    Serial.println("--- END BLOCK: paste over the defines of piece_thresholds.h ---");
}

/* Guided campaign walk: step prompts, stability capture,
 * auto-capture and the per-slot threshold row generation */
#include "hwtest.h"

/* Row generator, private to the guided walk */
void emitGameThresholds();

/* Game mode: guided calibration sequence. The operator is prompted for
 * each (piece, slot) combination and presses SPACE to record once the
 * reading stabilizes. The kGamePieces array lists what the walk visits:
 * the empty board plus the four played pieces. LOOP_CALL plays the band
 * below idle where the BACKWARD piece used to live.
 * kGameSlotCount is the number of slots covered (currently 12,
 * extensible to 20) */
constexpr uint8_t kGameSamplesPerCombo    = 8;
constexpr uint16_t kGameStabilitySpanMax  = 100;
constexpr uint8_t kGameStabilityReadCount = 5;

/* Walked pieces: empty board first, then the four played ones. The
 * backward band of the threshold table keeps no piece anymore; the loop
 * piece is a 1mm magnet south pole facing, read below idle */
const BoxPieceId kGamePieces[] = {
    BoxPieceId::EMPTY,      BoxPieceId::FORWARD,   BoxPieceId::LOOP_CALL,
    BoxPieceId::TURN_RIGHT, BoxPieceId::TURN_LEFT,
};

/* Entries the walk visits (empty + four played pieces) */
constexpr uint8_t kGamePieceCount = 5;

const char *kGamePieceHints[] = {
    "EMPTY (remove piece)",       "FORWARD (green, 4mm N)",  "LOOP_CALL (red, 1mm S)",
    "TURN_RIGHT (yellow, 1mm N)", "TURN_LEFT (blue, 4mm S)",
};

bool gGameActive         = false;
uint8_t gGamePieceIndex  = 0;
uint8_t gGameSlotIndex   = 0;
uint8_t gGameSampleIndex = 0;

/* Auto-capture: while true the game polls the current slot and records a
 * sample as soon as the reading settles, so the operator only places
 * pieces instead of pressing SPACE per sample. EMPTY is excluded (one
 * SPACE confirms the whole board is clear). Toggle with 'a' in game mode */
bool gGameAuto = true;

/* Set when a stable reading was rejected as an outlier, so auto-capture
 * warns once per sample instead of on every poll. Cleared on each advance */
bool gGameOutlierWarned = false;

// GAME MODE — guided piece calibration
// ==========================================

/* Total steps the game will walk through. Each (piece, slot) combo gets
 * kGameSamplesPerCombo recordings */
static uint16_t gameTotalSteps()
{
    return static_cast<uint16_t>(kGamePieceCount) * kGameSlotCount * kGameSamplesPerCombo;
}

/* Current linear step number (1-based) across the whole sequence */
static uint16_t gameCurrentStep()
{
    return static_cast<uint16_t>(gGamePieceIndex) * kGameSlotCount * kGameSamplesPerCombo +
           static_cast<uint16_t>(gGameSlotIndex) * kGameSamplesPerCombo +
           static_cast<uint16_t>(gGameSampleIndex) + 1;
}

void startGameMode()
{
    gGameActive        = true;
    gGamePieceIndex    = 0;
    gGameSlotIndex     = 0;
    gGameSampleIndex   = 0;
    gGameOutlierWarned = false;

    Serial.println("\n--- GAME MODE STARTED ---");
    Serial.print("   ");
    Serial.print(gameTotalSteps());
    Serial.print(" steps (");
    Serial.print(static_cast<int>(kGamePieceCount));
    Serial.print(" pieces x ");
    Serial.print(static_cast<int>(kGameSlotCount));
    Serial.print(" slots x ");
    Serial.print(static_cast<int>(kGameSamplesPerCombo));
    Serial.println(" samples)");
    Serial.print("   auto-capture ");
    Serial.println(gGameAuto ? "ON" : "OFF");
    Serial.println("   SPACE = record, 'a' = toggle auto, 'r' = retry, 'q' = quit");
    displayGameInstruction();
}

void stopGameMode()
{
    gGameActive = false;
    Serial.println("\n--- GAME MODE ENDED ---");
    emitGameThresholds();
}

void displayGameInstruction()
{
    const BoxPieceId piece = kGamePieces[gGamePieceIndex];

    /* EMPTY piece is special: record all slots at once since the board
     * typically starts empty. No need to confirm each slot individually */
    if (piece == BoxPieceId::EMPTY)
    {
        Serial.print("   [");
        Serial.print(gameCurrentStep());
        Serial.print("/");
        Serial.print(gameTotalSteps());
        Serial.print("] ");
        Serial.print(kGamePieceHints[gGamePieceIndex]);
        Serial.println(" for all slots");
        Serial.println("   -> confirm all slots empty, then SPACE");
        return;
    }

    Serial.print("   [");
    Serial.print(gameCurrentStep());
    Serial.print("/");
    Serial.print(gameTotalSteps());
    Serial.print("] slot ");
    Serial.print(static_cast<int>(gGameSlotIndex) + 1);
    Serial.print(", ");
    Serial.print(kGamePieceHints[gGamePieceIndex]);
    Serial.print(" [");
    Serial.print(static_cast<int>(gGameSampleIndex) + 1);
    Serial.print("/");
    Serial.print(static_cast<int>(kGameSamplesPerCombo));
    Serial.println("]");
    if (gGameAuto)
    {
        Serial.println("   -> place/adjust; auto-records when stable ('a' off, SPACE force)");
    }
    else
    {
        Serial.println("   -> place/adjust, wait for stability, then SPACE ('a' for auto)");
    }
}

/* Read one slot several times and check stability. If stable, write the
 * mean to outMean and return true. Otherwise print the span and return
 * false. Used by both normal recording and bulk EMPTY recording */
bool readSlotWithStability(uint8_t slotIndex, uint16_t &outMean)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    uint16_t minVal = 0xFFFF;
    uint16_t maxVal = 0;
    int32_t sum     = 0;

    for (uint8_t i = 0; i < kGameStabilityReadCount; ++i)
    {
        int16_t raw = 0;
        bool ok     = readAdsSingleEndedMean(&entry, channel, raw);
        if (!ok)
        {
            recoverAdsModule(&entry);
            ok = readAdsSingleEndedMean(&entry, channel, raw);
        }
        if (!ok)
        {
            Serial.print("   slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.println(" read FAIL");
            return false;
        }
        const uint16_t uval = static_cast<uint16_t>(raw);
        if (uval < minVal)
        {
            minVal = uval;
        }
        if (uval > maxVal)
        {
            maxVal = uval;
        }
        sum += raw;
    }

    const uint16_t span = maxVal - minVal;
    if (span > kGameStabilitySpanMax)
    {
        Serial.print("   slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        Serial.print(" unstable (span=");
        Serial.print(span);
        Serial.print(", min=");
        Serial.print(minVal);
        Serial.print(", max=");
        Serial.print(maxVal);
        Serial.println(")");
        return false;
    }

    outMean = static_cast<uint16_t>(sum / kGameStabilityReadCount);
    Serial.print("   slot ");
    Serial.print(static_cast<int>(slotIndex) + 1);
    Serial.print(" raw=");
    Serial.print(outMean);
    Serial.print(" (span=");
    Serial.print(span);
    Serial.println(")");
    return true;
}

/* Record EMPTY for all slots at once. Used when the board starts empty
 * to avoid tedious per-slot confirmation */
bool recordAllSlotsEmpty()
{
    Serial.println("   recording EMPTY for all slots...");

    for (uint8_t slot = 0; slot < kGameSlotCount; ++slot)
    {
        uint16_t mean = 0;
        if (!readSlotWithStability(slot, mean))
        {
            Serial.println("   retry SPACE");
            return false;
        }
        addLabelPoint(slot, BoxPieceId::EMPTY, mean);
    }

    return true;
}

/* True while a stable reading still looks like this slot's empty state, i.e.
 * the piece has not been placed yet. Auto-capture must not start sampling a
 * non-EMPTY piece before it is physically present: the slot is perfectly
 * stable while empty, so without this check the empty reading becomes the
 * first sample of the piece and the outlier guard cannot catch it (the
 * cluster is still empty, so it has nothing to compare against). The band is
 * this slot's own EMPTY reading +/- 100 counts, not a global threshold:
 * idle levels differ by ~900 counts across sensors. The margin stays small
 * because empty readings are stable to a few counts within a run and the
 * nearest piece (TURN_RIGHT) sits only ~250 counts above EMPTY. */
bool slotReadsEmpty(uint8_t slotIndex, BoxPieceId piece, uint16_t mean)
{
    if (piece == BoxPieceId::EMPTY)
    {
        return false; /* EMPTY is recorded on purpose by the operator */
    }
    const LabelCluster &empty = gPieceClusters[slotIndex][static_cast<uint8_t>(BoxPieceId::EMPTY)];
    if (empty.count == 0)
    {
        return false; /* no reference yet: fall back to the outlier guard */
    }
    constexpr uint16_t kMargin = 100;
    const uint32_t bottom      = empty.min > kMargin ? empty.min - kMargin : 0;
    const uint32_t top         = static_cast<uint32_t>(empty.max) + kMargin;
    return mean >= bottom && mean <= top;
}

/* Try to record one sample for (slot, piece): read with stability, apply
 * the outlier guard, then add the point. Returns the outcome instead of
 * a bool so callers choose their reaction. When verbose is false nothing
 * is printed for unstable/read-fail (the auto poll retries silently every
 * loop); an outlier still warns once per sample via gGameOutlierWarned */
GameSampleResult recordGameSample(uint8_t slotIndex, BoxPieceId piece, bool verbose)
{
    const AdsTestEntry &entry = kAdsTests[slotIndex / 4];
    const uint8_t channel     = slotIndex % 4;

    const LabelCluster &existing = gPieceClusters[slotIndex][static_cast<uint8_t>(piece)];

    /* The piece is placed once and then held still, so re-running the full
     * stability window for every sample would cost kGameStabilityReadCount
     * times the ADC time for no extra information: 12 samples would mean 12
     * times five mean reads on the same settled sensor. Only the first
     * sample of a combo (empty cluster) and every manual SPACE keep the full
     * window, because both follow a fresh placement of the piece. Later
     * samples take a single mean read and rely on the outlier guard below */
    const uint8_t readCount = (verbose || existing.count == 0) ? kGameStabilityReadCount : 1;

    uint16_t minVal = 0xFFFF;
    uint16_t maxVal = 0;
    int32_t sum     = 0;

    for (uint8_t i = 0; i < readCount; ++i)
    {
        int16_t raw = 0;
        bool ok     = readAdsSingleEndedMean(&entry, channel, raw);
        if (!ok)
        {
            recoverAdsModule(&entry);
            ok = readAdsSingleEndedMean(&entry, channel, raw);
        }
        if (!ok)
        {
            if (verbose)
            {
                Serial.println("   read FAIL, press SPACE again");
            }
            return GameSampleResult::ReadFail;
        }
        const uint16_t uval = static_cast<uint16_t>(raw);
        if (uval < minVal)
        {
            minVal = uval;
        }
        if (uval > maxVal)
        {
            maxVal = uval;
        }
        sum += raw;
    }

    const uint16_t span = maxVal - minVal;
    if (span > kGameStabilitySpanMax)
    {
        if (verbose)
        {
            Serial.print("   unstable (span=");
            Serial.print(span);
            Serial.print(", min=");
            Serial.print(minVal);
            Serial.print(", max=");
            Serial.print(maxVal);
            Serial.println("), press SPACE again");
        }
        return GameSampleResult::Unstable;
    }

    const uint16_t mean = static_cast<uint16_t>(sum / readCount);

    /* Auto capture only: while the slot still reads empty the piece is not
     * down yet, so waiting gives the operator the time to place it. SPACE
     * forces past this gate for a manual record */
    if (!verbose && slotReadsEmpty(slotIndex, piece, mean))
    {
        if (!gGameOutlierWarned)
        {
            Serial.print("   [WAIT] slot ");
            Serial.print(static_cast<int>(slotIndex) + 1);
            Serial.println(" still reads empty, place the piece (SPACE forces)");
            gGameOutlierWarned = true;
        }
        return GameSampleResult::PieceNotPlaced;
    }

    /* Outlier guard: same ±20% logic as recordLabelPoint, so a misplaced
     * piece cannot silently stretch the cluster */
    if (existing.count > 0)
    {
        const uint32_t spanBottom =
            existing.min > existing.min / 5 ? existing.min - existing.min / 5 : 0;
        const uint32_t spanTop = existing.max + existing.max / 5;
        if (mean < spanBottom || mean > spanTop)
        {
            /* Auto poll runs every loop: warn once per sample, not per poll */
            if (verbose || !gGameOutlierWarned)
            {
                Serial.print("   [OUTLIER IGNORED] raw=");
                Serial.print(mean);
                Serial.print(" expected ");
                Serial.print(static_cast<int>(spanBottom));
                Serial.print("-");
                Serial.println(static_cast<int>(spanTop));
            }
            gGameOutlierWarned = true;
            return GameSampleResult::Outlier;
        }
    }

    addLabelPoint(slotIndex, piece, mean);

    if (verbose)
    {
        Serial.print("   recorded raw=");
        Serial.print(mean);
        Serial.print(" (span=");
        Serial.print(span);
        Serial.print(", n=");
        Serial.print(existing.count);
        Serial.println(")");
    }
    return GameSampleResult::Recorded;
}

/* Auto-capture poll: attempt one sample for the current combo and advance
 * on success. Unstable/read-fail is silent (the piece is still settling,
 * the next loop retries); an outlier leaves the state untouched so the
 * operator can reseat the piece or press SPACE to force past it */
void pollGameAutoCapture()
{
    const uint8_t slotIndex = gGameSlotIndex;
    const BoxPieceId piece  = kGamePieces[gGamePieceIndex];

    const GameSampleResult result = recordGameSample(slotIndex, piece, false);
    if (result == GameSampleResult::Recorded)
    {
        Serial.print("   [auto] slot ");
        Serial.print(static_cast<int>(slotIndex) + 1);
        Serial.print(" ");
        Serial.print(labelName(static_cast<int>(piece)));
        Serial.print(" n=");
        Serial.println(gPieceClusters[slotIndex][static_cast<uint8_t>(piece)].count);
        gGameOutlierWarned = false;
        advanceGameState();
    }
}

void advanceGameState()
{
    const BoxPieceId currentPiece = kGamePieces[gGamePieceIndex];

    /* EMPTY is recorded in bulk for all slots at once, so skip all
     * EMPTY steps and jump directly to the next piece */
    if (currentPiece == BoxPieceId::EMPTY)
    {
        gGameSampleIndex = 0;
        gGameSlotIndex   = 0;
        ++gGamePieceIndex;
        if (gGamePieceIndex >= kGamePieceCount)
        {
            stopGameMode();
            return;
        }
        displayGameInstruction();
        return;
    }

    ++gGameSampleIndex;
    if (gGameSampleIndex >= kGameSamplesPerCombo)
    {
        gGameSampleIndex = 0;
        ++gGameSlotIndex;
        if (gGameSlotIndex >= kGameSlotCount)
        {
            gGameSlotIndex = 0;
            ++gGamePieceIndex;
            if (gGamePieceIndex >= kGamePieceCount)
            {
                stopGameMode();
                return;
            }
            Serial.println();
            Serial.print("=== NEW PIECE: ");
            Serial.print(kGamePieceHints[gGamePieceIndex]);
            Serial.println(" ===");
        }
        else
        {
            Serial.println();
            Serial.print("=== MOVE PIECE TO SLOT ");
            Serial.print(static_cast<int>(gGameSlotIndex) + 1);
            Serial.println(" ===");
        }
    }
    displayGameInstruction();
}

/* Compute a threshold row per slot from that slot's own clusters, print the
 * per-slot gap analysis and emit the pasteable table block. Each sensor idles
 * at its own level (12675..13616 counts on the 12-slot board), so a class gap
 * that is clean on one slot can overlap on the next: no global threshold can
 * separate the five classes on every slot at once. A slot whose classes do not
 * separate cleanly keeps an all-zero row, which decodes INVALID instead of
 * guessing a false piece */
void emitGameThresholds()
{
    static_assert(kCalibrationSlotCount == GASPETTO_ADC_SLOTS,
                  "per-slot table size must match the calibration slot count");

    /* Canonical ascending piece order: the first table columns follow it,
     * so a slot whose measured order differs would silently swap two
     * pieces. The band above FORWARD carries no piece */
    static const BoxPieceId kCanonicalOrder[] = {
        BoxPieceId::TURN_LEFT,  BoxPieceId::LOOP_CALL, BoxPieceId::EMPTY,
        BoxPieceId::TURN_RIGHT, BoxPieceId::FORWARD,
    };

    uint16_t slotRows[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS] = {};
    uint8_t slotsCalibrated                                  = 0;

    Serial.println("\n--- PER-SLOT THRESHOLD ANALYSIS ---");
    for (uint8_t slot = 0; slot < kCalibrationSlotCount; ++slot)
    {
        struct PieceRange
        {
            BoxPieceId piece;
            uint16_t min;
            uint16_t max;
            uint32_t count;
            int32_t mean;
        };
        PieceRange ranges[kGamePieceCount];
        uint8_t written = 0;

        for (uint8_t p = 0; p < kGamePieceCount; ++p)
        {
            const LabelCluster &cl = gPieceClusters[slot][static_cast<uint8_t>(kGamePieces[p])];
            if (cl.count == 0)
            {
                continue;
            }
            PieceRange &r = ranges[written];
            r.piece       = kGamePieces[p];
            r.min         = cl.min;
            r.max         = cl.max;
            r.count       = cl.count;
            r.mean        = static_cast<int32_t>(cl.sum / cl.count);
            ++written;
        }

        Serial.print("   slot ");
        Serial.print(static_cast<int>(slot) + 1);
        if (written == 0)
        {
            Serial.println("  [NO DATA]");
            continue;
        }
        if (written != kGamePieceCount)
        {
            Serial.println("  [INCOMPLETE, row zeroed]");
            continue;
        }

        /* Insertion sort by mean (5-6 elements) */
        for (uint8_t i = 1; i < written; ++i)
        {
            PieceRange tmp = ranges[i];
            uint8_t j      = i;
            while (j > 0 && ranges[j - 1].mean > tmp.mean)
            {
                ranges[j] = ranges[j - 1];
                --j;
            }
            ranges[j] = tmp;
        }

        bool orderOk = true;
        for (uint8_t i = 0; i < written; ++i)
        {
            if (ranges[i].piece != kCanonicalOrder[i])
            {
                orderOk = false;
                break;
            }
        }
        if (!orderOk)
        {
            Serial.println("  [BAD ORDER, row zeroed]");
            continue;
        }

        /* Each column holds the start of its piece: the first keeps its own
         * minimum, the next ones start at the midpoint of the two adjacent
         * ranges */
        uint16_t row[GASPETTO_ADC_COLS] = {};
        row[0]                          = ranges[0].min;
        int8_t overlapAt                = -1; /* upper piece of the first overlap */
        int32_t overlapGap              = 0;
        for (uint8_t i = 1; i < written; ++i)
        {
            const int32_t gap =
                static_cast<int32_t>(ranges[i].min) - static_cast<int32_t>(ranges[i - 1].max);
            row[i] = static_cast<uint16_t>(
                (static_cast<uint32_t>(ranges[i - 1].max) + ranges[i].min) / 2U);
            if (gap <= 0 && overlapAt < 0)
            {
                overlapAt  = static_cast<int8_t>(i);
                overlapGap = gap;
            }
        }

        if (overlapAt >= 0)
        {
            Serial.print("  [OVERLAP ");
            Serial.print(labelName(static_cast<int>(ranges[overlapAt - 1].piece)));
            Serial.print('/');
            Serial.print(labelName(static_cast<int>(ranges[overlapAt].piece)));
            Serial.print(" gap=");
            Serial.print(overlapGap);
            Serial.println(", row zeroed]");
            continue;
        }
        /* The band above FORWARD carries no piece: pin its start to a value
         * no reading can reach (raw is an int16_t mean), so the top of the
         * scale belongs to FORWARD and nothing decodes the old loop band */
        row[GASPETTO_ADC_SLOT_COL_LOOP_START] = 0xFFFFU;

        for (uint8_t c = 0; c < GASPETTO_ADC_COLS; ++c)
        {
            slotRows[slot][c] = row[c];
        }
        ++slotsCalibrated;

        /* Class ranges and the gaps between them, for a human check */
        Serial.print("  [OK] ");
        for (uint8_t i = 0; i < written; ++i)
        {
            Serial.print(labelName(static_cast<int>(ranges[i].piece)));
            Serial.print('[');
            Serial.print(ranges[i].min);
            Serial.print("..");
            Serial.print(ranges[i].max);
            Serial.print(i + 1 < written ? "] " : "] gaps ");
        }
        for (uint8_t i = 1; i < written; ++i)
        {
            const int32_t gap =
                static_cast<int32_t>(ranges[i].min) - static_cast<int32_t>(ranges[i - 1].max);
            Serial.print(gap);
            if (i + 1 < written)
            {
                Serial.print(',');
            }
        }
        Serial.println();
    }

    Serial.print("   calibrated slots: ");
    Serial.print(slotsCalibrated);
    Serial.print('/');
    Serial.println(kGameSlotCount);

    /* Emit the table between the piece_thresholds.h markers, so it pastes
     * over the placeholder rows in a single selection */
    Serial.println("\n--- GENERATED PER-SLOT ROWS ---");
    Serial.println("/* --- BEGIN GENERATED PER-SLOT ROWS --- */");
    Serial.println("static const uint16_t "
                   "GASPETTO_ADC_SLOT_THRESHOLDS[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS] = {");
    for (uint8_t slot = 0; slot < GASPETTO_ADC_SLOTS; ++slot)
    {
        Serial.print("    /* slot ");
        Serial.print(static_cast<int>(slot) + 1);
        Serial.print(" */ { ");
        for (uint8_t c = 0; c < GASPETTO_ADC_COLS; ++c)
        {
            Serial.print(slotRows[slot][c]);
            Serial.print(c + 1 < GASPETTO_ADC_COLS ? ", " : " }");
        }
        Serial.println(",");
    }
    Serial.println("};");
    Serial.println("/* --- END GENERATED PER-SLOT ROWS --- */");
}

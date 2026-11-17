/* Jig mode: measure the pieces once, measure the slots' gain, and derive the
 * per-slot threshold rows from both. The game walks slot by slot and needs
 * the piece in hand to change every time; this walk is type major, so one
 * piece stays in the operator's hand for a whole pass, and it never asks for
 * a keypress: the reading itself says when a piece is in and when it settled.
 *
 * Two quantities only. A piece is characterised by its deviation from the
 * slot's idle level, and a slot by the gain that scales every deviation.
 * Both are ratios, so the rail's own wander cancels instead of being
 * measured: the deviation scales with the supply exactly like the idle it is
 * taken against, which is why the numbers survive a session restart.
 *
 * The scan's warm up pass has no equivalent here and does not need one: a
 * scan compares absolute levels taken minutes apart, so the first pass after
 * a reset reads low by the board's own warm up, while every deviation below
 * is taken against an idle level measured seconds earlier in the same slot.
 * What does need care is the reverse case: a census spread over several
 * minutes would compare readings taken against a stale idle, so the
 * reference slot's idle is re-measured at the start of each type, and the
 * gain pass re-measures it in every slot including the reference one */
#include "hwtest.h"

#include <EEPROM.h>

/* Played pieces in campaign order: the 4 mm magnets first, because their
 * larger field gives the best signal to noise and a wrong magnet in that
 * batch shows up within the first minutes */
constexpr uint8_t kJigTypeCount = 4;

static const BoxPieceId kJigTypes[kJigTypeCount] = {
    BoxPieceId::FORWARD,
    BoxPieceId::TURN_LEFT,
    BoxPieceId::TURN_RIGHT,
    BoxPieceId::LOOP_CALL,
};

/* Colour of each type. The operator handles the pieces by colour, so the
 * walk speaks in colours and never asks anyone to read a piece name; the
 * bench firmware owns the mapping from a colour to the band the shared table
 * describes. The bands ascend with the raw reading, and this batch reads the
 * greens lowest and the blues highest, which is why the band order below is
 * not the historical one: kBandPieces in scan.cpp carries the same mapping */
static const char *const kJigTypeHints[kJigTypeCount] = {
    "green 4mm",  /* FORWARD: band 0, the most negative */
    "blue 4mm",   /* TURN_LEFT: band 4, the most positive */
    "yellow 1mm", /* TURN_RIGHT: band 1, the weak negative */
    "red 1mm",    /* LOOP_CALL: band 3, the weak positive */
};

/* Ascending class order the threshold table is built on: the row lists the
 * start of each band in this order, so a slot whose measured centres sort
 * differently would silently swap two pieces */
constexpr uint8_t kJigClassCount = 5;

/* Type index of each band in ascending order; 0xFF is the EMPTY band, which
 * has no piece and sits at the slot's own idle level. The bands ascend with
 * the raw reading, and this batch puts the greens at the bottom and the blues
 * at the top, so the order below is the reverse of the historical one and
 * matches kBandPieces in scan.cpp */
static const uint8_t kJigClassType[kJigClassCount] = {0, 2, 0xFF, 3, 1};

/* Pieces per type, orientations per piece, slots to walk */
constexpr uint8_t kJigPiecesPerType = 5;

/* A type is still usable with fewer than five pieces: a broken one is
 * declared with 'n' and the plan stops asking for it. Three is the floor
 * where a median and a spread still mean something */
constexpr uint8_t kJigMinPieces = 3;

/* Poses the operator declared unavailable, remembered so the walk finishes
 * instead of asking for them forever */
constexpr uint8_t kJigMaxSkips    = 32;
constexpr uint8_t kJigOrientCount = 4;
constexpr uint8_t kJigSlotCount   = kCalibrationSlotCount;

/* The census happens in one slot; the gain pass walks them all, the
 * reference one included so both ends of the ratio are measured at the same
 * moment */
constexpr uint8_t kJigReferenceSlot = 0;

static_assert(kJigSlotCount == GASPETTO_ADC_SLOTS, "jig slots must match the shared table");

/* Store keys that are not a type index */
constexpr uint8_t kJigKeySlotIdle = 0xFE; /* idle taken for the gain pass in a slot */

/* Detector thresholds, in counts. The weakest piece reads about 600 counts
 * away from idle, so 200 separates it from an empty slot with margin while
 * still catching a piece landing slightly off centre */
constexpr uint16_t kJigPresentDelta = 200;

/* A plateau is six readings whose span stays inside 40 counts: one reading
 * is a fifteen sample mean already, and 40 counts is under 2 % of the
 * weakest piece, so the window can lock on a reading that drifts by a few
 * counts while the piece settles. A wild reading leaves the window after six
 * polls instead of poisoning it */
constexpr uint8_t kJigStableCount = 6;
constexpr uint16_t kJigStableSpan = 40;

/* How far a reading may sit from the expected idle and still be accepted as
 * an idle level. A piece left in the slot reads hundreds of counts away, so
 * this refuses it instead of turning a piece into the slot's reference. The
 * slots idle up to 550 counts apart on this board, hence the margin */
constexpr uint16_t kJigIdlePlausible = 900;

/* Rejections of the idle before the firmware says what it is waiting for: a
 * silent loop would look like a frozen board */
constexpr uint8_t kJigIdleRejectHint = 10;

/* A new capture in the same slot needs the reading to have moved from the
 * one just recorded. Arming on a return to idle was the first design and it
 * missed the moment a piece is lifted and turned: a reading costs the rail's
 * settle time, so a quick lift is invisible and the walk stalled.
 *
 * The threshold is only slightly above the jitter, which is what two
 * identical placements show (0 to 15 counts). An earlier value of 150
 * confused "the piece did not move" with "the piece rotated and the reading
 * moved by less than 150", which happens whenever the magnet sits close to
 * the sensor's null axis: a 48-count change is a real turn and has to pass.
 *
 * A truly centred magnet reads the same in every orientation and would block
 * forever, so after this many refused polls the capture is taken anyway and
 * the log says only what was observed */
constexpr uint16_t kJigRearmDelta = 30;
constexpr uint8_t kJigStuckPolls  = 8;

/* A pair of bands closer than this cannot survive the rail's wander and a
 * piece landing a little off: the slot is reported as tight and the sensor's
 * position becomes the lever */
constexpr int32_t kJigTightGap = 300;

/* A slot whose gain sits this far (per mille) from the median is reported:
 * too low and its bands squeeze against the empty one, too high and the
 * strongest piece approaches the sensor's ceiling */
constexpr int32_t kJigGainSpreadPermille = 300;

/* Half-width added for the play in the slot, in per mille of the deviation.
 * The piece can land anywhere within that play, and the field falls with the
 * cube of the distance, so the error is proportional to the deviation, not a
 * fixed count. The 120 comes from the 588-count spread measured over four
 * orientations on a piece reading about 2700: that spread is the full width,
 * so half of it is about 11 %. Replace it with the 8-placement measurement
 * once it is done: four orientations at a fixed position, two positions */
constexpr int32_t kJigPlayPermille = 120;

/* Empty class half width: twice the rail wander measured on the bench, which
 * is the whole reason the empty band needs to be wider than a few counts */
constexpr uint16_t kJigEmptyHalf = 100;

/* Rotation counts for a type when the four orientations move the reading by
 * more than this: below it the magnet is centred enough that the choice of
 * edge buys nothing, and asking four placements per piece would only waste
 * the operator's time */
constexpr uint16_t kJigRotationSignificant = 50;

/* Pieces used to decide whether rotations matter for the rest of the type */
constexpr uint8_t kJigRotationProbePieces = 3;

/* A piece this far from its type's median is worth a look: at the magnet
 * sizes used here it is about the difference between two magnet lengths */
constexpr int32_t kJigSuspectPermille = 200;

constexpr uint16_t kJigRoundStep = 5;

/* Room for the whole campaign plus redoes: repeating a placement replaces
 * the capture it supersedes */
constexpr uint16_t kJigMaxCaptures = 240;

enum class JigKind : uint8_t
{
    Idle,   /* idle level of a slot: the reference every deviation uses */
    Census, /* one piece, one orientation, in the reference slot */
    Gain,   /* the reference piece in one slot: the slot's gain */
};

struct JigCapture
{
    JigKind kind;
    uint8_t type;
    uint8_t piece;
    uint8_t orient;
    uint8_t slot;
    int16_t mean;
    uint16_t ref;
};

struct JigTarget
{
    JigKind kind;
    uint8_t type;
    uint8_t piece;
    uint8_t orient;
    uint8_t slot;
};

struct JigTypeStats
{
    bool complete;
    int32_t median;   /* deviation of the type at gain 1, counts */
    int32_t maxDev;   /* farthest piece from that median */
    uint8_t refPiece; /* piece closest to the median, carries the gain pass */
    bool rotationMatters;
};

bool jig_active = false;

static JigCapture captures[kJigMaxCaptures] = {};
static uint16_t capture_count               = 0;
static bool ref_known[kJigSlotCount]        = {};
static uint16_t ref_value[kJigSlotCount]    = {};

/* Arm state of the detector: a capture drops the arm, and the reading has to
 * move away from the value just recorded before another one is accepted,
 * which covers both lifting the piece and turning it in place */
static bool has_captured      = false;
static int16_t captured_value = 0;
static uint8_t captured_slot  = 0;

/* Force-capture request from the shell, consumed by the next poll: a piece
 * too weak to cross the presence threshold still has to be recordable */
static bool capture_now = false;

/* Current plateau window: the last few readings, with the oldest dropped as
 * new ones arrive. A plain accumulator was the first design and it never
 * recovered: once one reading widened the span past the tolerance, the oldest
 * extreme stayed in it forever and the capture could not happen again. A
 * window keeps only recent readings, so a piece that drifts while settling
 * locks as soon as it holds still */
static int16_t window[kJigStableCount] = {};
static uint8_t window_count            = 0;
static uint8_t window_next             = 0;

/* Defined further down: the flash journal needs the store, and the store
 * reports to the journal after every change, so the two refer to each other */
static void jigRebuildRefs();
static void jigStoreMaybeSave();
static void jigStoreLoad();
static void jigStoreSave();

/* Target on screen, so the prompt is printed only when it changes */
static bool target_printed    = false;
static JigTarget shown_target = {};

static bool analysis_done = false;

/* Consecutive refused idle readings, only to explain the wait */
static uint8_t idle_rejects = 0;

/* Two idle measurements of the same slot taken minutes apart should agree to
 * a few tens of counts: the rail wanders by that much, not by hundreds. A
 * larger spread means one of them caught a piece next to the slot or a hand
 * over it, and the deviations of that family are shifted by as much */
constexpr int32_t kJigIdleSuspect    = 200;
constexpr uint8_t kJigMaxIdlePerSlot = 12;

/* Consecutive polls where the reading had not moved away from the previous
 * capture: a centred magnet never does, so the guard gives way after a few */
static uint8_t unchanged_polls = 0;

/* Last census piece measured, so 'n' redoes that one and not the piece the
 * prompt has already moved on to */
static bool last_census_valid    = false;
static uint8_t last_census_type  = 0;
static uint8_t last_census_piece = 0;

/* ========================================== */
/* Capture store */
/* ========================================== */

static bool jigSameKey(const JigCapture &capture, JigKind kind, uint8_t type, uint8_t piece,
                       uint8_t orient, uint8_t slot)
{
    return capture.kind == kind && capture.type == type && capture.piece == piece &&
           capture.orient == orient && capture.slot == slot;
}

static const JigCapture *jigFind(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient,
                                 uint8_t slot)
{
    /* Newest wins: a re-placed piece replaces the capture it supersedes */
    for (int32_t index = static_cast<int32_t>(capture_count) - 1; index >= 0; --index)
    {
        if (jigSameKey(captures[index], kind, type, piece, orient, slot))
        {
            return &captures[index];
        }
    }
    return nullptr;
}

static void jigRecord(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot,
                      int16_t mean, uint16_t ref)
{
    if (capture_count >= kJigMaxCaptures)
    {
        Serial.println("   [store full, clear with 'X']");
        return;
    }

    JigCapture &capture = captures[capture_count++];
    capture.kind        = kind;
    capture.type        = type;
    capture.piece       = piece;
    capture.orient      = orient;
    capture.slot        = slot;
    capture.mean        = mean;
    capture.ref         = ref;

    if (kind == JigKind::Idle)
    {
        ref_known[slot] = true;
        ref_value[slot] = static_cast<uint16_t>(mean);
    }

    jigStoreMaybeSave();
}

struct JigSkip
{
    JigKind kind;
    uint8_t type;
    uint8_t piece;
    uint8_t orient;
    uint8_t slot;
};

static JigSkip skips[kJigMaxSkips] = {};
static uint8_t skip_count          = 0;

static bool jigIsSkipped(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot)
{
    for (uint8_t index = 0; index < skip_count; ++index)
    {
        const JigSkip &skip = skips[index];
        if (skip.kind == kind && skip.type == type && skip.piece == piece &&
            skip.orient == orient && skip.slot == slot)
        {
            return true;
        }
    }
    return false;
}

static void jigSkipAdd(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot)
{
    if (skip_count >= kJigMaxSkips)
    {
        Serial.println("   [skip list full]");
        return;
    }

    JigSkip &skip = skips[skip_count++];
    skip.kind     = kind;
    skip.type     = type;
    skip.piece    = piece;
    skip.orient   = orient;
    skip.slot     = slot;

    jigStoreMaybeSave();
}

/* A capture counts as present when it was recorded or explicitly skipped */
static bool jigMissing(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot)
{
    return jigFind(kind, type, piece, orient, slot) == nullptr &&
           !jigIsSkipped(kind, type, piece, orient, slot);
}

static uint16_t jigRefFor(uint8_t slot)
{
    return ref_known[slot] ? ref_value[slot] : 0;
}

static int32_t jigDeviation(const JigCapture *capture)
{
    return static_cast<int32_t>(capture->mean) - static_cast<int32_t>(capture->ref);
}

static int32_t jigGap(int32_t left, int32_t right)
{
    return left > right ? left - right : right - left;
}

/* ========================================== */
/* Flash journal */
/* ========================================== */
/* A campaign outlives a reset only if it is on flash, and losing twenty
 * minutes of placement to a stray reset is the whole reason this exists. The
 * part has no EEPROM, so the core emulates one in its last flash page and
 * exposes a RAM buffer: fill() reads the page, buffered writes go to RAM and
 * flush() erases the page and writes the buffer back.
 *
 * That flush erases a 128 kB sector here, about a second, so it does not
 * happen on every capture: a save every few poses bounds what a reset can
 * cost to the last handful of placements, and everything that ends a session
 * (stop, clear, forget a slot, restart a piece) saves straight away */
constexpr uint8_t kJigStoreMagicA     = 0x4A; /* 'J' */
constexpr uint8_t kJigStoreMagicB     = 0x47; /* 'G' */
constexpr uint8_t kJigStoreVersion    = 1;
constexpr uint16_t kJigStoreHeader    = 8;
constexpr uint8_t kJigStoreRecordSize = 12;
constexpr uint16_t kJigStoreCapacity  = kJigMaxCaptures + kJigMaxSkips;
constexpr uint16_t kJigStoreTotal     = kJigStoreHeader + kJigStoreCapacity * kJigStoreRecordSize;
constexpr uint8_t kJigStoreKindSkip   = 3;
constexpr uint8_t kJigSaveEvery       = 4;

static_assert(kJigStoreTotal <= E2END + 1, "the journal must fit the emulated EEPROM");

static uint8_t unsaved    = 0;
static bool store_ready   = false;
static bool store_loading = false;

static void jigStoreWriteU8(uint16_t position, uint8_t value)
{
    eeprom_buffered_write_byte(position, value);
}

static void jigStoreWriteU16(uint16_t position, uint16_t value)
{
    jigStoreWriteU8(position, static_cast<uint8_t>(value & 0xFFU));
    jigStoreWriteU8(static_cast<uint16_t>(position + 1), static_cast<uint8_t>(value >> 8));
}

static uint8_t jigStoreReadU8(uint16_t position)
{
    return eeprom_buffered_read_byte(position);
}

static uint16_t jigStoreReadU16(uint16_t position)
{
    return static_cast<uint16_t>(
        jigStoreReadU8(position) |
        (static_cast<uint16_t>(jigStoreReadU8(static_cast<uint16_t>(position + 1))) << 8));
}

/* One record covers both a capture and a write-off: the kind byte says which,
 * and byte 5 carries the target kind a write-off applies to */
static void jigStoreWriteRecord(uint16_t index, uint8_t kind, uint8_t type, uint8_t piece,
                                uint8_t orient, uint8_t slot, uint8_t extra, int16_t mean,
                                uint16_t ref)
{
    const uint16_t base = static_cast<uint16_t>(kJigStoreHeader + index * kJigStoreRecordSize);

    jigStoreWriteU8(base, kind);
    jigStoreWriteU8(static_cast<uint16_t>(base + 1), type);
    jigStoreWriteU8(static_cast<uint16_t>(base + 2), piece);
    jigStoreWriteU8(static_cast<uint16_t>(base + 3), orient);
    jigStoreWriteU8(static_cast<uint16_t>(base + 4), slot);
    jigStoreWriteU8(static_cast<uint16_t>(base + 5), extra);
    jigStoreWriteU16(static_cast<uint16_t>(base + 6), static_cast<uint16_t>(mean));
    jigStoreWriteU16(static_cast<uint16_t>(base + 8), ref);
    jigStoreWriteU16(static_cast<uint16_t>(base + 10), 0);
}

static void jigStoreSave()
{
    if (store_loading)
    {
        return;
    }

    uint16_t index = 0;

    jigStoreWriteU8(0, kJigStoreMagicA);
    jigStoreWriteU8(1, kJigStoreMagicB);
    jigStoreWriteU8(2, kJigStoreVersion);
    jigStoreWriteU16(3, static_cast<uint16_t>(capture_count + skip_count));

    for (uint16_t captureIndex = 0; captureIndex < capture_count && index < kJigStoreCapacity;
         ++captureIndex)
    {
        const JigCapture &capture = captures[captureIndex];
        jigStoreWriteRecord(index++, static_cast<uint8_t>(capture.kind), capture.type,
                            capture.piece, capture.orient, capture.slot, 0, capture.mean,
                            capture.ref);
    }
    for (uint8_t skipIndex = 0; skipIndex < skip_count && index < kJigStoreCapacity; ++skipIndex)
    {
        const JigSkip &skip = skips[skipIndex];
        jigStoreWriteRecord(index++, kJigStoreKindSkip, skip.type, skip.piece, skip.orient,
                            skip.slot, static_cast<uint8_t>(skip.kind), 0, 0);
    }

    /* Blank the rest of the journal: a shorter save must not leave the tail
     * of a longer one behind, since the count alone would not catch it */
    for (; index < kJigStoreCapacity; ++index)
    {
        jigStoreWriteRecord(index, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0);
    }

    eeprom_buffer_flush();

    unsaved     = 0;
    store_ready = true;

    Serial.print("   [flash ");
    Serial.print(capture_count);
    Serial.println(" captures]");
}

static void jigStoreMaybeSave()
{
    if (store_loading)
    {
        return;
    }
    if (unsaved < 0xFF)
    {
        ++unsaved;
    }
    if (unsaved >= kJigSaveEvery)
    {
        jigStoreSave();
    }
}

static void jigStoreLoad()
{
    if (store_ready)
    {
        return;
    }
    store_ready = true;

    eeprom_buffer_fill();

    if (jigStoreReadU8(0) != kJigStoreMagicA || jigStoreReadU8(1) != kJigStoreMagicB ||
        jigStoreReadU8(2) != kJigStoreVersion)
    {
        return; /* blank page, or content from another firmware */
    }

    const uint16_t stored = jigStoreReadU16(3);
    if (stored == 0 || stored > kJigStoreCapacity)
    {
        return;
    }

    store_loading = true;
    for (uint16_t index = 0; index < stored; ++index)
    {
        const uint16_t base = static_cast<uint16_t>(kJigStoreHeader + index * kJigStoreRecordSize);
        const uint8_t kind  = jigStoreReadU8(base);

        if (kind == kJigStoreKindSkip)
        {
            jigSkipAdd(static_cast<JigKind>(jigStoreReadU8(static_cast<uint16_t>(base + 5))),
                       jigStoreReadU8(static_cast<uint16_t>(base + 1)),
                       jigStoreReadU8(static_cast<uint16_t>(base + 2)),
                       jigStoreReadU8(static_cast<uint16_t>(base + 3)),
                       jigStoreReadU8(static_cast<uint16_t>(base + 4)));
        }
        else if (kind <= 2)
        {
            jigRecord(static_cast<JigKind>(kind), jigStoreReadU8(static_cast<uint16_t>(base + 1)),
                      jigStoreReadU8(static_cast<uint16_t>(base + 2)),
                      jigStoreReadU8(static_cast<uint16_t>(base + 3)),
                      jigStoreReadU8(static_cast<uint16_t>(base + 4)),
                      static_cast<int16_t>(jigStoreReadU16(static_cast<uint16_t>(base + 6))),
                      jigStoreReadU16(static_cast<uint16_t>(base + 8)));
        }
    }
    store_loading = false;

    jigRebuildRefs();

    Serial.print("   resumed ");
    Serial.print(capture_count);
    Serial.print(" captures and ");
    Serial.print(skip_count);
    Serial.println(" write-offs from flash");
}

/* ========================================== */
/* Plan: the first capture still missing */
/* ========================================== */

/* A type stops asking for four orientations once the first few pieces show
 * the rotation does not move the reading by more than the significance
 * threshold; deciding earlier would cost placements, deciding later would
 * cost four times as many */
static bool jigRotationMatters(uint8_t type)
{
    int32_t lowest  = 0;
    int32_t highest = 0;
    uint8_t pieces  = 0;

    for (uint8_t piece = 0; piece < kJigPiecesPerType && pieces < kJigRotationProbePieces; ++piece)
    {
        int32_t pieceLow  = 0;
        int32_t pieceHigh = 0;
        bool any          = false;

        for (uint8_t orient = 0; orient < kJigOrientCount; ++orient)
        {
            const JigCapture *capture =
                jigFind(JigKind::Census, type, piece, orient, kJigReferenceSlot);
            if (capture == nullptr)
            {
                continue;
            }
            const int32_t dev = jigDeviation(capture);
            if (!any)
            {
                pieceLow  = dev;
                pieceHigh = dev;
                any       = true;
            }
            else
            {
                pieceLow  = dev < pieceLow ? dev : pieceLow;
                pieceHigh = dev > pieceHigh ? dev : pieceHigh;
            }
        }

        if (!any)
        {
            continue; /* skipped or not measured yet: says nothing either way */
        }
        if (pieces == 0)
        {
            lowest  = pieceLow;
            highest = pieceHigh;
        }
        else
        {
            lowest  = pieceLow < lowest ? pieceLow : lowest;
            highest = pieceHigh > highest ? pieceHigh : highest;
        }
        ++pieces;
    }

    if (pieces < 2)
    {
        return true; /* one piece is not a sample: keep asking for four */
    }

    return (highest - lowest) > static_cast<int32_t>(kJigRotationSignificant);
}

static uint8_t jigOrientationsFor(uint8_t type)
{
    return jigRotationMatters(type) ? kJigOrientCount : 1;
}

/* The gain pass uses one piece per type, and always in orientation one: the
 * ratio only needs the same edge at both ends, and tying it to a fixed
 * orientation keeps the plan and the analysis looking up the same key. The
 * piece closest to the type's median is the least likely to sit at a band
 * edge and skew the scaling */
static uint8_t jigRefPiece(uint8_t type)
{
    int32_t devs[kJigPiecesPerType]    = {};
    uint8_t used                       = 0;
    uint8_t pieceOf[kJigPiecesPerType] = {};

    for (uint8_t piece = 0; piece < kJigPiecesPerType; ++piece)
    {
        const JigCapture *capture = jigFind(JigKind::Census, type, piece, 0, kJigReferenceSlot);
        if (capture == nullptr)
        {
            continue;
        }
        devs[used]    = jigDeviation(capture);
        pieceOf[used] = piece;
        ++used;
    }

    if (used == 0)
    {
        return 0;
    }

    for (uint8_t i = 1; i < used; ++i)
    {
        const int32_t held     = devs[i];
        const uint8_t keyPiece = pieceOf[i];
        uint8_t j              = i;
        while (j > 0 && devs[j - 1] > held)
        {
            devs[j]    = devs[j - 1];
            pieceOf[j] = pieceOf[j - 1];
            --j;
        }
        devs[j]    = held;
        pieceOf[j] = keyPiece;
    }

    return pieceOf[used / 2];
}

static bool jigNextTarget(JigTarget &target)
{
    /* Census: the reference slot's idle first, then five pieces per type in
     * every orientation still worth asking for. The idle is re-measured per
     * type so a census spread over several minutes never compares a reading
     * against an idle taken ten minutes earlier */
    for (uint8_t type = 0; type < kJigTypeCount; ++type)
    {
        if (jigMissing(JigKind::Idle, type, 0, 0, kJigReferenceSlot))
        {
            target = {JigKind::Idle, type, 0, 0, kJigReferenceSlot};
            return true;
        }

        const uint8_t orientations = jigOrientationsFor(type);
        for (uint8_t piece = 0; piece < kJigPiecesPerType; ++piece)
        {
            for (uint8_t orient = 0; orient < orientations; ++orient)
            {
                if (jigMissing(JigKind::Census, type, piece, orient, kJigReferenceSlot))
                {
                    target = {JigKind::Census, type, piece, orient, kJigReferenceSlot};
                    return true;
                }
            }
        }
    }

    /* Gains: one piece of one type across the board. The gain is a property
     * of the sensor's geometry, so a single type scales the four the same
     * way and twenty placements are enough instead of eighty. Every slot gets
     * a fresh idle first, the reference one included, so both ends of the
     * ratio are measured in the same pass */
    const uint8_t refPiece = jigRefPiece(0);
    for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
    {
        if (jigMissing(JigKind::Idle, kJigKeySlotIdle, 0, 0, slot))
        {
            target = {JigKind::Idle, kJigKeySlotIdle, 0, 0, slot};
            return true;
        }
        if (jigMissing(JigKind::Gain, 0, refPiece, 0, slot))
        {
            target = {JigKind::Gain, 0, refPiece, 0, slot};
            return true;
        }
    }

    return false;
}

/* ========================================== */
/* Reading and detector */
/* ========================================== */

static bool jigReadSlot(uint8_t slot, int16_t &mean)
{
    const AdsTestEntry &entry = kAdsTests[slot / 4];
    const uint8_t channel     = static_cast<uint8_t>(slot % 4);

    if (readAdsSingleEndedMean(&entry, channel, mean))
    {
        return true;
    }
    recoverAdsModule(&entry);
    return readAdsSingleEndedMean(&entry, channel, mean);
}

static void jigPlateauReset()
{
    window_count = 0;
    window_next  = 0;
}

static void jigPlateauAdd(int16_t mean)
{
    window[window_next] = mean;
    window_next         = static_cast<uint8_t>((window_next + 1) % kJigStableCount);
    if (window_count < kJigStableCount)
    {
        ++window_count;
    }
}

static bool jigPlateauStable()
{
    if (window_count < kJigStableCount)
    {
        return false;
    }

    int16_t lowest  = window[0];
    int16_t highest = window[0];
    for (uint8_t index = 1; index < kJigStableCount; ++index)
    {
        lowest  = window[index] < lowest ? window[index] : lowest;
        highest = window[index] > highest ? window[index] : highest;
    }
    return static_cast<uint16_t>(highest - lowest) <= kJigStableSpan;
}

static int16_t jigPlateauMean()
{
    int32_t sum = 0;
    for (uint8_t index = 0; index < window_count; ++index)
    {
        sum += window[index];
    }
    return static_cast<int16_t>(sum / static_cast<int32_t>(window_count));
}

/* ========================================== */
/* Prompts */
/* ========================================== */

static const char *jigOrientationHint(uint8_t orient)
{
    switch (orient)
    {
    case 0:
        return "text readable";
    case 1:
        return "text 90 cw";
    case 2:
        return "text upside down";
    default:
        return "text 270 cw";
    }
}

static void jigPrintProgress()
{
    uint16_t censusDone = 0;
    uint16_t censusPlan = 0;
    uint16_t gainDone   = 0;

    for (uint8_t type = 0; type < kJigTypeCount; ++type)
    {
        const uint8_t orientations = jigOrientationsFor(type);
        censusPlan += static_cast<uint16_t>(kJigPiecesPerType) * orientations;
        for (uint8_t piece = 0; piece < kJigPiecesPerType; ++piece)
        {
            for (uint8_t orient = 0; orient < orientations; ++orient)
            {
                if (jigFind(JigKind::Census, type, piece, orient, kJigReferenceSlot) != nullptr)
                {
                    ++censusDone;
                }
            }
        }
    }
    for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
    {
        if (jigFind(JigKind::Gain, 0, jigRefPiece(0), 0, slot) != nullptr)
        {
            ++gainDone;
        }
    }

    Serial.print("   progress: census ");
    Serial.print(censusDone);
    Serial.print('/');
    Serial.print(censusPlan);
    Serial.print(", gains ");
    Serial.print(gainDone);
    Serial.print('/');
    Serial.print(kJigSlotCount);
    Serial.println("   [q stop, d dump]");
}

static void jigPrintTarget(const JigTarget &target)
{
    Serial.println();

    switch (target.kind)
    {
    case JigKind::Idle:
        Serial.print("   [idle] slot ");
        Serial.print(static_cast<int>(target.slot) + 1);
        if (target.type != kJigKeySlotIdle)
        {
            Serial.print(" before ");
            Serial.print(kJigTypeHints[target.type]);
        }
        Serial.println(": leave it empty, holding still");
        break;

    case JigKind::Census: {
        Serial.print("   [census] ");
        Serial.print(kJigTypeHints[target.type]);
        Serial.print(", piece ");
        Serial.print(static_cast<int>(target.piece) + 1);
        Serial.print('/');
        Serial.print(kJigPiecesPerType);
        Serial.print(", orientation ");
        Serial.print(static_cast<int>(target.orient) + 1);
        Serial.print('/');
        Serial.print(jigOrientationsFor(target.type));
        Serial.print(" (");
        Serial.print(jigOrientationHint(target.orient));
        Serial.print(") in slot ");
        Serial.print(kJigReferenceSlot + 1);
        Serial.println(": place it");
        break;
    }

    case JigKind::Gain: {
        Serial.print("   [gain] slot ");
        Serial.print(static_cast<int>(target.slot) + 1);
        Serial.print('/');
        Serial.print(kJigSlotCount);
        Serial.print(": empty first, then ");
        Serial.print(kJigTypeHints[0]);
        Serial.print(", piece ");
        Serial.print(static_cast<int>(target.piece) + 1);
        Serial.println(" (text readable)");
        break;
    }
    }

    jigPrintProgress();
}

/* Which edge of the piece to paint, from the orientation that won. The
 * analysis names the winning pose as "text upside down"; that is not an
 * instruction anybody can act on, so the edge is named relative to the text
 * itself, which the operator can always read at a glance */
static const char *jigMarkEdgeHint(uint8_t orient)
{
    switch (orient)
    {
    case 0:
        return "paint the edge at the top of the text";
    case 1:
        return "paint the edge at the left of the text";
    case 2:
        return "paint the edge at the bottom of the text";
    default:
        return "paint the edge at the right of the text";
    }
}

/* ========================================== */
/* Analysis */
/* ========================================== */

static uint16_t jigRound5(int32_t value)
{
    if (value <= 0)
    {
        return 0;
    }
    if (value >= 65530)
    {
        return 65530;
    }
    return static_cast<uint16_t>(((value + (kJigRoundStep / 2)) / kJigRoundStep) * kJigRoundStep);
}

static void jigSort(int32_t *values, uint8_t count)
{
    for (uint8_t i = 1; i < count; ++i)
    {
        const int32_t held = values[i];
        uint8_t j          = i;
        while (j > 0 && values[j - 1] > held)
        {
            values[j] = values[j - 1];
            --j;
        }
        values[j] = held;
    }
}

static int32_t jigMedian(const int32_t *sorted, uint8_t count)
{
    return count == 0 ? 0 : sorted[count / 2];
}

/* Deviation of one piece, taking the orientation closest to the type's
 * centre: an off-centre magnet reads differently a quarter turn apart, and
 * the edge the operator marks is exactly that best orientation */
static bool jigPieceDeviation(uint8_t type, uint8_t piece, int32_t centre, int32_t &deviation)
{
    bool any        = false;
    int32_t best    = 0;
    int32_t bestGap = 0;

    for (uint8_t orient = 0; orient < kJigOrientCount; ++orient)
    {
        const JigCapture *capture =
            jigFind(JigKind::Census, type, piece, orient, kJigReferenceSlot);
        if (capture == nullptr)
        {
            continue;
        }
        const int32_t dev = jigDeviation(capture);
        const int32_t gap = jigGap(dev, centre);
        if (!any || gap < bestGap)
        {
            best    = dev;
            bestGap = gap;
            any     = true;
        }
    }

    deviation = best;
    return any;
}

static void jigAnalysePieces(JigTypeStats *stats)
{
    Serial.println("\n--- PIECE ANALYSIS (reference slot 1) ---");

    for (uint8_t type = 0; type < kJigTypeCount; ++type)
    {
        int32_t all[kJigPiecesPerType * kJigOrientCount] = {};
        uint8_t allCount                                 = 0;

        for (uint8_t piece = 0; piece < kJigPiecesPerType; ++piece)
        {
            for (uint8_t orient = 0; orient < kJigOrientCount; ++orient)
            {
                const JigCapture *capture =
                    jigFind(JigKind::Census, type, piece, orient, kJigReferenceSlot);
                if (capture == nullptr)
                {
                    continue;
                }
                all[allCount++] = jigDeviation(capture);
            }
        }

        /* Pieces actually measured: a broken one is declared with 'n', so a
         * type can be complete with fewer than five and still give a median */
        uint8_t piecesSeen = 0;
        for (uint8_t piece = 0; piece < kJigPiecesPerType; ++piece)
        {
            for (uint8_t orient = 0; orient < kJigOrientCount; ++orient)
            {
                if (jigFind(JigKind::Census, type, piece, orient, kJigReferenceSlot) != nullptr)
                {
                    ++piecesSeen;
                    break;
                }
            }
        }

        stats[type].complete        = piecesSeen >= kJigMinPieces;
        stats[type].rotationMatters = jigRotationMatters(type);

        Serial.print("  ");
        Serial.print(kJigTypeHints[type]);
        Serial.print("  ");
        Serial.print(piecesSeen);
        Serial.print('/');
        Serial.print(kJigPiecesPerType);
        Serial.print(" pieces  as ");
        Serial.print(labelName(static_cast<int>(kJigTypes[type])));

        if (!stats[type].complete)
        {
            Serial.println("  [INCOMPLETE]");
            continue;
        }

        jigSort(all, allCount);
        const int32_t centre = jigMedian(all, allCount);

        int32_t pieceDev[kJigPiecesPerType] = {};
        uint8_t used                        = 0;
        for (uint8_t piece = 0; piece < kJigPiecesPerType; ++piece)
        {
            int32_t dev = 0;
            if (jigPieceDeviation(type, piece, centre, dev))
            {
                pieceDev[used++] = dev;
            }
        }

        jigSort(pieceDev, used);
        const int32_t median = jigMedian(pieceDev, used);

        int32_t maxDev = 0;
        for (uint8_t index = 0; index < used; ++index)
        {
            const int32_t gap = jigGap(pieceDev[index], median);
            maxDev            = gap > maxDev ? gap : maxDev;
        }

        stats[type].median   = median;
        stats[type].maxDev   = maxDev;
        stats[type].refPiece = jigRefPiece(type);

        Serial.print("  median ");
        if (median >= 0)
        {
            Serial.print('+');
        }
        Serial.print(median);
        Serial.print(" spread ");
        Serial.print(maxDev);
        Serial.print(" rotation ");
        Serial.println(stats[type].rotationMatters ? "matters" : "irrelevant");

        const int32_t magnitude = jigGap(median, 0);
        const int32_t suspect   = (magnitude * kJigSuspectPermille) / 1000;

        for (uint8_t piece = 0; piece < kJigPiecesPerType; ++piece)
        {
            int32_t dev = 0;
            if (!jigPieceDeviation(type, piece, centre, dev))
            {
                continue;
            }
            const int32_t gap  = jigGap(dev, median);
            uint8_t bestOrient = 0;
            int32_t bestGap    = 0;
            bool orientKnown   = false;
            int32_t orientLow  = 0;
            int32_t orientHigh = 0;

            for (uint8_t orient = 0; orient < kJigOrientCount; ++orient)
            {
                const JigCapture *capture =
                    jigFind(JigKind::Census, type, piece, orient, kJigReferenceSlot);
                if (capture == nullptr)
                {
                    continue;
                }
                const int32_t orientDev = jigDeviation(capture);
                const int32_t orientGap = jigGap(orientDev, dev);
                if (!orientKnown)
                {
                    orientLow  = orientDev;
                    orientHigh = orientDev;
                }
                else
                {
                    orientLow  = orientDev < orientLow ? orientDev : orientLow;
                    orientHigh = orientDev > orientHigh ? orientDev : orientHigh;
                }
                if (!orientKnown || orientGap < bestGap)
                {
                    bestOrient  = orient;
                    bestGap     = orientGap;
                    orientKnown = true;
                }
            }

            Serial.print("    piece ");
            Serial.print(static_cast<int>(piece) + 1);
            Serial.print(" dev ");
            if (dev >= 0)
            {
                Serial.print('+');
            }
            Serial.print(dev);
            /* What the rotation actually bought on this piece: the log no
             * longer claims a magnet is centred, this is the measurement that
             * says whether it is */
            Serial.print(" rotation spread ");
            Serial.print(orientHigh - orientLow);
            Serial.print(" mark: ");
            Serial.print(jigOrientationHint(bestOrient));
            Serial.print(", ");
            Serial.print(jigMarkEdgeHint(bestOrient));
            if (gap > suspect)
            {
                Serial.print("  << off the median by ");
                Serial.print(gap);
                Serial.print(", check the magnet");
            }
            Serial.println();
        }

        Serial.print("    gain piece: ");
        Serial.print(static_cast<int>(stats[type].refPiece) + 1);
        Serial.println();
    }
}

/* The table's column order is a convention the production decode relies on:
 * FORWARD < TURN_RIGHT < EMPTY < LOOP_CALL < TURN_LEFT. A piece built with
 * the other pole lands on the wrong side of the idle, and every slot would
 * then be reported as an overlap without ever saying why */
static bool jigOrderOk(const JigTypeStats *stats)
{
    for (uint8_t type = 0; type < kJigTypeCount; ++type)
    {
        if (!stats[type].complete)
        {
            return false;
        }
    }

    return (stats[0].median < stats[2].median) && (stats[2].median < 0) && (stats[3].median > 0) &&
           (stats[1].median > stats[3].median);
}

static void jigPrintOrder(const JigTypeStats *stats)
{
    int32_t sorted[kJigTypeCount] = {};
    uint8_t order[kJigTypeCount]  = {};

    for (uint8_t type = 0; type < kJigTypeCount; ++type)
    {
        sorted[type] = stats[type].median;
        order[type]  = type;
    }

    for (uint8_t i = 1; i < kJigTypeCount; ++i)
    {
        const int32_t held    = sorted[i];
        const uint8_t heldKey = order[i];
        uint8_t j             = i;
        while (j > 0 && sorted[j - 1] > held)
        {
            sorted[j] = sorted[j - 1];
            order[j]  = order[j - 1];
            --j;
        }
        sorted[j] = held;
        order[j]  = heldKey;
    }

    Serial.println("\n--- CANONICAL ORDER CHECK ---");
    Serial.print("  measured, ascending: ");
    for (uint8_t i = 0; i < kJigTypeCount; ++i)
    {
        Serial.print(kJigTypeHints[order[i]]);
        Serial.print(" as ");
        Serial.print(labelName(static_cast<int>(kJigTypes[order[i]])));
        Serial.print('(');
        if (sorted[i] >= 0)
        {
            Serial.print('+');
        }
        Serial.print(sorted[i]);
        Serial.print(')');
        Serial.print(i + 1 < kJigTypeCount ? "  " : "");
    }
    Serial.println();
    Serial.println("  expected           : TURN_LEFT < LOOP_CALL < EMPTY < TURN_RIGHT < FORWARD");
    Serial.println("  >> two families sit on the wrong side of the idle for this table.");
    Serial.println("     Swap the labels of the reversed pair, or flip their magnets.");
    Serial.println("     No rows are emitted until the measured order matches.");
}

/* Which piece the gain pass actually used, read back from the records. The
 * ratio only needs the same piece at both ends, so the analysis must follow
 * the captures rather than the piece the census happens to name: redo the
 * census, the named piece changes, and a lookup keyed to it would orphan a
 * gain pass that is still perfectly valid */
static bool jigGainPiece(uint8_t &piece)
{
    for (uint16_t index = 0; index < capture_count; ++index)
    {
        if (captures[index].kind == JigKind::Gain)
        {
            piece = captures[index].piece;
            return true;
        }
    }
    return false;
}

/* Gain of a slot in thousandths: the same piece's deviation in that slot over
 * its deviation in the reference slot, both measured in the gain pass with
 * their own idle. Returns 0 when either end is missing */
static int32_t jigGainMilli(uint8_t slot, uint8_t refPiece)
{
    const JigCapture *base = jigFind(JigKind::Gain, 0, refPiece, 0, kJigReferenceSlot);
    if (base == nullptr)
    {
        return 0;
    }
    const int32_t baseDev = jigDeviation(base);
    if (baseDev == 0)
    {
        return 0;
    }

    const JigCapture *capture = jigFind(JigKind::Gain, 0, refPiece, 0, slot);
    if (capture == nullptr)
    {
        return 0;
    }
    return static_cast<int32_t>((static_cast<int64_t>(jigDeviation(capture)) * 1000) / baseDev);
}

static void jigEmitRows(const JigTypeStats *stats)
{
    uint16_t rows[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS];
    uint16_t calibrated = 0;
    uint8_t refPiece    = stats[0].refPiece;

    /* Follow the gain pass rather than the census: a census redone after a
     * piece is re-glued names a different reference, and the gains already
     * measured stay valid because the ratio uses one piece at both ends */
    if (jigGainPiece(refPiece))
    {
        Serial.print("\n   gain pass measured with ");
        Serial.print(kJigTypeHints[0]);
        Serial.print(" piece ");
        Serial.println(static_cast<int>(refPiece) + 1);
    }

    /* Seed from the compiled table: a slot the run could not trust keeps the
     * row it already had instead of dropping to a row of zeros */
    for (uint8_t slot = 0; slot < GASPETTO_ADC_SLOTS; ++slot)
    {
        for (uint8_t column = 0; column < GASPETTO_ADC_COLS; ++column)
        {
            rows[slot][column] = GASPETTO_ADC_SLOT_THRESHOLDS[slot][column];
        }
    }

    /* Gains first: the verification sheet targets the median gain, so the
     * rows cannot all be built before it is known */
    int32_t gains[kJigSlotCount]       = {};
    int32_t sortedGains[kJigSlotCount] = {};
    uint8_t gainCount                  = 0;
    for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
    {
        gains[slot] = jigGainMilli(slot, refPiece);
        if (gains[slot] != 0)
        {
            sortedGains[gainCount++] = gains[slot];
        }
    }
    jigSort(sortedGains, gainCount);
    const int32_t medianGain = jigMedian(sortedGains, gainCount);

    bool flagged[kJigSlotCount] = {};

    Serial.println("\n--- SLOT GAINS ---");

    for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
    {
        const uint16_t ref = jigRefFor(slot);
        const int32_t gain = gains[slot];

        Serial.print("  slot ");
        if (slot < 9)
        {
            Serial.print(' ');
        }
        Serial.print(static_cast<int>(slot) + 1);
        Serial.print("  ref ");
        Serial.print(ref);
        Serial.print("  gain ");
        Serial.print(gain / 1000);
        Serial.print('.');
        Serial.print((gain % 1000) / 10);
        Serial.print("  ");

        if (ref == 0 || gain == 0 || !stats[0].complete)
        {
            Serial.println("[NO DATA, compiled row kept]");
            continue;
        }

        int32_t centre[kJigClassCount] = {};
        int32_t half[kJigClassCount]   = {};

        for (uint8_t cls = 0; cls < kJigClassCount; ++cls)
        {
            const uint8_t type = kJigClassType[cls];
            if (type == 0xFF)
            {
                centre[cls] = ref;
                half[cls]   = kJigEmptyHalf;
                continue;
            }
            if (!stats[type].complete)
            {
                centre[cls] = ref;
                half[cls]   = 0;
                continue;
            }
            /* The play adds its share to the half width. A piece can land
             * anywhere within the slot's play, and the field error that
             * follows is proportional to the deviation, so the allowance
             * scales with it instead of being a fixed count */
            const int32_t play = static_cast<int32_t>(
                (static_cast<int64_t>(jigGap(stats[type].median, 0)) * kJigPlayPermille) / 1000);

            centre[cls] =
                static_cast<int32_t>(ref) +
                static_cast<int32_t>((static_cast<int64_t>(stats[type].median) * gain) / 1000);
            half[cls] = static_cast<int32_t>(
                (static_cast<int64_t>(stats[type].maxDev + play) * gain) / 1000);
        }

        /* The bands must ascend in the canonical order and must not touch:
         * either failure means this slot cannot separate the pieces, and a
         * row guessed there would invent a piece that is not in the box */
        bool usable      = true;
        int32_t smallest = 0;
        for (uint8_t cls = 1; cls < kJigClassCount; ++cls)
        {
            const int32_t gap = (centre[cls] - half[cls]) - (centre[cls - 1] + half[cls - 1]);
            if (gap <= 0)
            {
                usable = false;
                break;
            }
            if (cls == 1 || gap < smallest)
            {
                smallest = gap;
            }
        }
        if (!usable)
        {
            flagged[slot] = true;
            Serial.println("[OVERLAP, compiled row kept]");
            continue;
        }

        /* Reported when the margins are too thin for the rail's wander and a
         * piece landing off centre, or when this sensor sees a clearly
         * different field than the others: both are fixed on the bench, not
         * by a threshold */
        const bool gainOff = (medianGain != 0) && (jigGap(gain, medianGain) * 1000 >
                                                   medianGain * kJigGainSpreadPermille);
        if (smallest < kJigTightGap || gainOff)
        {
            flagged[slot] = true;
        }

        uint16_t row[GASPETTO_ADC_COLS] = {};
        row[0]                          = jigRound5(centre[0] - half[0]);
        for (uint8_t cls = 1; cls < kJigClassCount; ++cls)
        {
            row[cls] =
                jigRound5(((centre[cls - 1] + half[cls - 1]) + (centre[cls] - half[cls])) / 2);
        }
        row[GASPETTO_ADC_COLS - 1] = 0xFFFFU;

        bool monotonic = true;
        for (uint8_t column = 1; column < GASPETTO_ADC_COLS - 1; ++column)
        {
            if (row[column] <= row[column - 1])
            {
                monotonic = false;
                break;
            }
        }
        if (!monotonic)
        {
            Serial.println("[BAD ORDER, compiled row kept]");
            continue;
        }

        for (uint8_t column = 0; column < GASPETTO_ADC_COLS; ++column)
        {
            rows[slot][column] = row[column];
        }
        ++calibrated;

        Serial.print("smallest gap ");
        Serial.print(smallest);
        Serial.print("  { ");
        for (uint8_t column = 0; column < GASPETTO_ADC_COLS; ++column)
        {
            Serial.print(row[column]);
            Serial.print(column + 1 < GASPETTO_ADC_COLS ? ", " : " }");
        }
        if (flagged[slot])
        {
            Serial.print("  << tight or weak: see the sheet");
        }
        Serial.println();
    }

    Serial.print("\n   computed slots: ");
    Serial.print(calibrated);
    Serial.print('/');
    Serial.println(kJigSlotCount);

    /* Verification sheet: only the slots that need bench work, and only the
     * target value. The target uses the median gain, not the slot's own, so
     * it names the reading to aim for instead of describing the fault: with
     * the slot's own gain the sheet would always agree with what is read,
     * and moving the sensor would have no visible goal */
    Serial.println("\n--- VERIFICATION SHEET (targets at the median gain) ---");
    if (medianGain == 0)
    {
        Serial.println("   no gain pass in this run: nothing to verify");
    }
    else
    {
        bool anyFlagged = false;

        Serial.print("   median gain ");
        Serial.print(medianGain / 1000);
        Serial.print('.');
        Serial.println((medianGain % 1000) / 10);

        for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
        {
            if (!flagged[slot])
            {
                continue;
            }
            anyFlagged = true;

            const uint16_t ref = jigRefFor(slot);

            Serial.print("  slot ");
            if (slot < 9)
            {
                Serial.print(' ');
            }
            Serial.print(static_cast<int>(slot) + 1);
            Serial.print("  gain ");
            Serial.print(gains[slot] / 1000);
            Serial.print('.');
            Serial.print((gains[slot] % 1000) / 10);
            Serial.print("  ");
            if (gains[slot] == 0)
            {
                Serial.println("no gain measured");
                continue;
            }
            if (gains[slot] < medianGain)
            {
                Serial.println("reads low: bring the sensor closer");
            }
            else if (gains[slot] > medianGain)
            {
                Serial.println("reads high: move the sensor away");
            }
            else
            {
                Serial.println("gain is fine: the pieces themselves are too close here");
            }

            Serial.print("            idle ");
            Serial.print(ref);
            for (uint8_t cls = 0; cls < kJigClassCount; ++cls)
            {
                const uint8_t type = kJigClassType[cls];
                int32_t centre     = static_cast<int32_t>(ref);
                int32_t half       = kJigEmptyHalf;

                if (type != 0xFF && stats[type].complete)
                {
                    const int32_t play = static_cast<int32_t>(
                        (static_cast<int64_t>(jigGap(stats[type].median, 0)) * kJigPlayPermille) /
                        1000);
                    centre = static_cast<int32_t>(ref) +
                             static_cast<int32_t>(
                                 (static_cast<int64_t>(stats[type].median) * medianGain) / 1000);
                    half   = static_cast<int32_t>(
                        (static_cast<int64_t>(stats[type].maxDev + play) * medianGain) / 1000);
                }

                Serial.print(" | ");
                if (type == 0xFF)
                {
                    Serial.print("EMPTY");
                }
                else
                {
                    Serial.print(labelName(static_cast<int>(kJigTypes[type])));
                    Serial.print('/');
                    Serial.print(kJigTypeHints[type]);
                }
                Serial.print(' ');
                Serial.print(centre);
                Serial.print(" +-");
                Serial.print(half);
            }
            Serial.println();
        }

        if (!anyFlagged)
        {
            Serial.println("   every slot is inside its margins");
        }
    }

    Serial.println("\n--- GENERATED PER-SLOT ROWS ---");
    Serial.println("/* --- BEGIN GENERATED PER-SLOT ROWS --- */");
    Serial.println("static const uint16_t "
                   "GASPETTO_ADC_SLOT_THRESHOLDS[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS] = {");
    for (uint8_t slot = 0; slot < GASPETTO_ADC_SLOTS; ++slot)
    {
        Serial.print("    /* slot ");
        Serial.print(static_cast<int>(slot) + 1);
        Serial.print(" */ { ");
        for (uint8_t column = 0; column < GASPETTO_ADC_COLS; ++column)
        {
            Serial.print(rows[slot][column]);
            Serial.print(column + 1 < GASPETTO_ADC_COLS ? ", " : " }");
        }
        Serial.println(",");
    }
    Serial.println("};");
    Serial.println("/* --- END GENERATED PER-SLOT ROWS --- */");
}

/* Every idle measurement grouped by slot, with its distance to that slot's
 * median. Two measurements of one slot minutes apart that disagree by
 * hundreds of counts cannot both be true, so this is where a bad reference
 * shows up before it has quietly shifted a whole family */
static void jigPrintIdleHistory()
{
    bool any = false;

    Serial.println("\n--- IDLE HISTORY (counts, deltas to the slot median) ---");

    for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
    {
        int32_t values[kJigMaxIdlePerSlot] = {};
        int32_t sorted[kJigMaxIdlePerSlot] = {};
        uint8_t count                      = 0;

        for (uint16_t index = 0; index < capture_count && count < kJigMaxIdlePerSlot; ++index)
        {
            const JigCapture &capture = captures[index];
            if (capture.kind == JigKind::Idle && capture.slot == slot)
            {
                values[count] = capture.mean;
                sorted[count] = capture.mean;
                ++count;
            }
        }

        if (count < 2)
        {
            continue; /* a single measurement has nothing to disagree with */
        }
        any = true;

        jigSort(sorted, count);
        const int32_t median = jigMedian(sorted, count);

        Serial.print("  slot ");
        if (slot < 9)
        {
            Serial.print(' ');
        }
        Serial.print(static_cast<int>(slot) + 1);
        Serial.print("  median ");
        Serial.print(median);
        Serial.print("  ");
        for (uint8_t index = 0; index < count; ++index)
        {
            const int32_t delta = values[index] - median;
            if (delta >= 0)
            {
                Serial.print('+');
            }
            Serial.print(delta);
            if (jigGap(values[index], median) > kJigIdleSuspect)
            {
                Serial.print(" <<");
            }
            Serial.print(index + 1 < count ? ", " : "");
        }
        Serial.println();
    }

    if (!any)
    {
        Serial.println("   no slot was measured more than once");
    }
}

static void jigAnalyse()
{
    JigTypeStats stats[kJigTypeCount] = {};

    jigPrintIdleHistory();
    jigAnalysePieces(stats);

    if (!jigOrderOk(stats))
    {
        jigPrintOrder(stats);
        analysis_done = true;
        printJigHelp();
        return;
    }

    jigEmitRows(stats);

    analysis_done = true;
    Serial.println("\n   'j' reprints, 'd' dumps the captures, 'X' clears and restarts");
    printJigHelp();
}

/* ========================================== */
/* Shell entry points */
/* ========================================== */

/* The whole key list, printed by the shell's help and by '?' inside the
 * mode: one source, so a new key cannot show up in one and not the other */
void printJigHelp()
{
    Serial.println("   jig:   j = start or resume the piece census and the slot gains");
    Serial.println("          SPACE = force a capture (a settled plateau is still required)");
    Serial.println("          d = dump every capture, one line each");
    Serial.println("          n = restart the piece just measured (its poses are dropped)");
    Serial.println("          b = undo the last capture, its pose comes back on screen");
    Serial.println("          q or x = stop the walk, captures kept in RAM");
    Serial.println("          N = write off the piece on screen: broken or lost, it takes four");
    Serial.println("          X = clear the captures and stop");
    Serial.println("          19k = forget slot 19, so the plan asks for it again");
    Serial.println("          ? = this help");
}

void dumpJigCaptures()
{
    Serial.print("\n--- JIG CAPTURES (");
    Serial.print(capture_count);
    Serial.println(") ---");

    for (uint16_t index = 0; index < capture_count; ++index)
    {
        const JigCapture &capture = captures[index];
        switch (capture.kind)
        {
        case JigKind::Idle:
            Serial.print("  idle   slot ");
            Serial.print(static_cast<int>(capture.slot) + 1);
            Serial.print(" ref ");
            Serial.print(capture.ref);
            Serial.print(" mean ");
            Serial.println(capture.mean);
            break;
        case JigKind::Census:
            Serial.print("  census ");
            Serial.print(kJigTypeHints[capture.type]);
            Serial.print(" piece ");
            Serial.print(static_cast<int>(capture.piece) + 1);
            Serial.print(" orient ");
            Serial.print(static_cast<int>(capture.orient) + 1);
            Serial.print(" dev ");
            Serial.print(jigDeviation(&capture));
            Serial.print(" ref ");
            Serial.println(capture.ref);
            break;
        case JigKind::Gain:
            Serial.print("  gain   slot ");
            Serial.print(static_cast<int>(capture.slot) + 1);
            Serial.print(" piece ");
            Serial.print(static_cast<int>(capture.piece) + 1);
            Serial.print(" dev ");
            Serial.print(jigDeviation(&capture));
            Serial.print(" ref ");
            Serial.println(capture.ref);
            break;
        }
    }
}

void startJigMode()
{
    if (test_mode != TestMode::Calibration)
    {
        Serial.println("jig mode works in calibration mode ('c' to switch)");
        return;
    }

    /* Whatever a reset left behind comes back before the plan is computed,
     * so the walk resumes on the pose it stopped at */
    jigStoreLoad();

    if (analysis_done)
    {
        jigAnalyse();
        return;
    }

    jig_active     = true;
    target_printed = false;
    has_captured   = false;
    jigPlateauReset();

    Serial.println("\n--- JIG MODE ---");
    if (capture_count > 0)
    {
        Serial.print("   resuming: ");
        Serial.print(capture_count);
        Serial.println(" captures in RAM");
    }
    Serial.println("   no keypress needed: place a piece and wait for the capture");
    printJigHelp();

    JigTarget target;
    if (!jigNextTarget(target))
    {
        jigAnalyse();
        jig_active = false;
    }
}

void stopJigMode()
{
    if (!jig_active)
    {
        return;
    }
    jig_active = false;
    jigStoreSave();
    Serial.print("\n--- JIG MODE stopped, ");
    Serial.print(capture_count);
    Serial.println(" captures kept ---");
    jigPrintProgress();
}

void clearJigCaptures()
{
    capture_count = 0;
    skip_count    = 0;
    analysis_done = false;
    has_captured  = false;
    for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
    {
        ref_known[slot] = false;
        ref_value[slot] = 0;
    }
    jigStoreSave();
    Serial.println("   jig captures cleared");
}

void forgetJigSlot(uint8_t slot)
{
    uint16_t kept = 0;

    /* Compact the store in place: the order only matters for the dump, and
     * the lookups take the newest match, so keeping the relative order of
     * what survives is enough */
    for (uint16_t index = 0; index < capture_count; ++index)
    {
        if (captures[index].slot != slot)
        {
            captures[kept++] = captures[index];
        }
    }

    Serial.print("   slot ");
    Serial.print(static_cast<int>(slot) + 1);
    Serial.print(": ");
    Serial.print(capture_count - kept);
    Serial.println(" captures forgotten, the plan will ask for them again");

    capture_count   = kept;
    ref_known[slot] = false;
    ref_value[slot] = 0;
    analysis_done   = false;
    has_captured    = false;
    jigStoreSave();
}

/* Writing off the pose on screen: a piece that is broken or lost takes all
 * four of its orientations at once, a single pose only itself */
static void jigSkipShown()
{
    if (!target_printed)
    {
        Serial.println("   no pose on screen to write off");
        return;
    }

    if (shown_target.kind == JigKind::Census)
    {
        for (uint8_t orient = 0; orient < kJigOrientCount; ++orient)
        {
            jigSkipAdd(JigKind::Census, shown_target.type, shown_target.piece, orient,
                       shown_target.slot);
        }
        Serial.print("   piece ");
        Serial.print(static_cast<int>(shown_target.piece) + 1);
        Serial.print(" written off: ");
        Serial.println(kJigTypeHints[shown_target.type]);
    }
    else
    {
        jigSkipAdd(shown_target.kind, shown_target.type, shown_target.piece, shown_target.orient,
                   shown_target.slot);
        Serial.println("   pose written off");
    }

    target_printed = false;
    jigPlateauReset();
}

/* The idle level of a slot is the last one recorded for it, so any change to
 * the store has to rebuild them: after an undo or a restart, the stale value
 * would be the one from the dropped capture */
static void jigRebuildRefs()
{
    for (uint8_t slot = 0; slot < kJigSlotCount; ++slot)
    {
        ref_known[slot] = false;
        ref_value[slot] = 0;
    }

    for (uint16_t index = 0; index < capture_count; ++index)
    {
        const JigCapture &capture = captures[index];
        if (capture.kind == JigKind::Idle)
        {
            ref_known[capture.slot] = true;
            ref_value[capture.slot] = static_cast<uint16_t>(capture.mean);
        }
    }
}

/* Redoing a piece: its poses are dropped and the walk starts it again at
 * orientation 1. Everything else is kept, so a piece placed badly costs four
 * poses, not the whole census. It targets the piece just measured rather than
 * the one on screen: by the time a mistake is noticed the prompt has already
 * moved to the next piece, and that is the piece nobody wants to redo */
static void jigRestartPiece(uint8_t type, uint8_t piece)
{
    uint16_t kept = 0;
    for (uint16_t index = 0; index < capture_count; ++index)
    {
        const JigCapture &capture = captures[index];
        if (capture.kind == JigKind::Census && capture.type == type && capture.piece == piece)
        {
            continue;
        }
        captures[kept++] = capture;
    }
    const uint16_t dropped = capture_count - kept;
    capture_count          = kept;

    /* A write-off goes with it, otherwise the piece would never come back */
    uint8_t keptSkips = 0;
    for (uint8_t index = 0; index < skip_count; ++index)
    {
        const JigSkip &skip = skips[index];
        if (skip.kind == JigKind::Census && skip.type == type && skip.piece == piece)
        {
            continue;
        }
        skips[keptSkips++] = skip;
    }
    skip_count = keptSkips;

    jigRebuildRefs();

    Serial.print("   ");
    Serial.print(kJigTypeHints[type]);
    Serial.print(" piece ");
    Serial.print(static_cast<int>(piece) + 1);
    Serial.print(" restarted, ");
    Serial.print(dropped);
    Serial.println(" poses dropped");

    analysis_done     = false;
    has_captured      = false;
    target_printed    = false;
    last_census_valid = false;
    jigStoreSave();
    jigPlateauReset();
}

/* Stepping back one capture: the pose comes back on screen and the operator
 * can place it again. Finer than restarting a whole piece when only the last
 * orientation was captured badly */
static void jigUndoLast()
{
    if (capture_count == 0)
    {
        Serial.println("   nothing recorded yet");
        return;
    }

    const JigCapture last = captures[--capture_count];

    Serial.print("   undone: ");
    switch (last.kind)
    {
    case JigKind::Idle:
        Serial.print("idle of slot ");
        Serial.println(static_cast<int>(last.slot) + 1);
        break;
    case JigKind::Census:
        Serial.print(kJigTypeHints[last.type]);
        Serial.print(" piece ");
        Serial.print(static_cast<int>(last.piece) + 1);
        Serial.print(" orient ");
        Serial.println(static_cast<int>(last.orient) + 1);
        break;
    case JigKind::Gain:
        Serial.print("gain of slot ");
        Serial.print(static_cast<int>(last.slot) + 1);
        Serial.print(", piece ");
        Serial.println(static_cast<int>(last.piece) + 1);
        break;
    }

    if (last.kind == JigKind::Census)
    {
        last_census_valid = false;
    }
    jigRebuildRefs();

    analysis_done  = false;
    has_captured   = false;
    target_printed = false;
    jigStoreSave();
    jigPlateauReset();
}

void runJigKey(char key)
{
    switch (key)
    {
    case ' ':
        capture_now = true;
        Serial.println("   [capture on the next settled reading]");
        break;
    case 'd':
        dumpJigCaptures();
        break;
    case 'n':
        if (last_census_valid)
        {
            jigRestartPiece(last_census_type, last_census_piece);
        }
        else if (target_printed && shown_target.kind == JigKind::Census)
        {
            jigRestartPiece(shown_target.type, shown_target.piece);
        }
        else
        {
            Serial.println("   no census piece to restart yet");
        }
        break;
    case 'b':
        jigUndoLast();
        break;
    case 'N':
        jigSkipShown();
        break;
    case 'j':
    case 'q':
    case 'x':
        stopJigMode();
        break;
    case 'X':
        clearJigCaptures();
        stopJigMode();
        break;
    case 'k':
        Serial.println("   stop first ('q'), then type the slot number and k, e.g. 19k");
        break;
    case '?':
        printJigHelp();
        break;
    default:
        break;
    }
}

void pollJig()
{
    JigTarget target;
    if (!jigNextTarget(target))
    {
        if (!analysis_done)
        {
            jigAnalyse();
        }
        jig_active = false;
        return;
    }

    /* The prompt follows the target: printed once, then left alone while the
     * operator handles the piece */
    if (!target_printed || target.kind != shown_target.kind || target.type != shown_target.type ||
        target.piece != shown_target.piece || target.orient != shown_target.orient ||
        target.slot != shown_target.slot)
    {
        /* A capture taken in another slot says nothing about the plateau in
         * this one, so the comparison value is dropped on a slot change;
         * inside a slot it survives, which is what keeps a placement that has
         * not moved from being recorded twice */
        if (!has_captured || captured_slot != target.slot)
        {
            has_captured = false;
        }
        jigPrintTarget(target);
        shown_target    = target;
        target_printed  = true;
        idle_rejects    = 0;
        unchanged_polls = 0;
        jigPlateauReset();
    }

    int16_t mean = 0;
    if (!jigReadSlot(target.slot, mean))
    {
        Serial.println("   read FAIL, retrying");
        return;
    }

    if (target.kind == JigKind::Idle)
    {
        /* An idle level becomes the reference every deviation in that slot is
         * taken against, so a piece still sitting there would poison the
         * whole slot. The reading has to land where an idle can: near the
         * level this slot last showed, or near the reference slot's own */
        const uint16_t expected = ref_known[target.slot] ? ref_value[target.slot] :
                                                           jigRefFor(kJigReferenceSlot);
        if (!capture_now && expected != 0 &&
            jigGap(mean, expected) > static_cast<int32_t>(kJigIdlePlausible))
        {
            jigPlateauReset();
            /* Silence would look like a hang: say what is refused and how to
             * override it, once every few seconds */
            if (idle_rejects < 0xFF)
            {
                ++idle_rejects;
            }
            if (idle_rejects == kJigIdleRejectHint)
            {
                Serial.print("   idle reads ");
                Serial.print(mean);
                Serial.print(", expected about ");
                Serial.print(expected);
                Serial.println(": remove the piece, or SPACE to force it");
            }
            return; /* probably a piece still in the slot: wait for it to go */
        }
        idle_rejects = 0;

        /* An idle level is the reference every other capture is taken
         * against, so it is recorded from a settled plateau and never from a
         * single reading - SPACE only lifts the plausibility guard, it does
         * not let a moving reading through */
        jigPlateauAdd(mean);
        if (!jigPlateauStable())
        {
            return;
        }

        const int16_t value = jigPlateauMean();
        jigRecord(JigKind::Idle, target.type, 0, 0, target.slot, value,
                  static_cast<uint16_t>(value));
        Serial.print("   idle measured: ");
        Serial.println(value);

        has_captured    = true;
        captured_value  = value;
        captured_slot   = target.slot;
        unchanged_polls = 0;
        capture_now     = false;
        target_printed  = false;
        jigPlateauReset();
        return;
    }

    const uint16_t ref = jigRefFor(target.slot);
    if (ref == 0)
    {
        Serial.println("   no idle reference for this slot, redo it with k<slot>");
        return;
    }

    if (!capture_now)
    {
        /* A piece has to be present... */
        if (jigGap(mean, ref) < static_cast<int32_t>(kJigPresentDelta))
        {
            jigPlateauReset();
            return;
        }
        /* ...and the reading has to have moved since the last capture, since
         * a plateau that has not changed is the one already recorded. A
         * centred magnet never moves, so after a few refusals the capture is
         * taken anyway and the prompt says the value did not change */
        if (has_captured && captured_slot == target.slot &&
            jigGap(mean, captured_value) < static_cast<int32_t>(kJigRearmDelta))
        {
            if (unchanged_polls < 0xFF)
            {
                ++unchanged_polls;
            }
            if (unchanged_polls < kJigStuckPolls)
            {
                jigPlateauReset();
                return;
            }
            if (unchanged_polls == kJigStuckPolls)
            {
                Serial.print("   reading did not move by ");
                Serial.print(kJigRearmDelta);
                Serial.println(" counts: recording it as it stands");
            }
        }
        else
        {
            unchanged_polls = 0;
        }
    }

    jigPlateauAdd(mean);
    if (!jigPlateauStable())
    {
        return;
    }

    const int16_t value    = jigPlateauMean();
    const int32_t captured = static_cast<int32_t>(value) - static_cast<int32_t>(ref);

    jigRecord(target.kind, target.type, target.piece, target.orient, target.slot, value, ref);

    Serial.print("   captured ");
    Serial.print(target.kind == JigKind::Census ? "census" : "gain");
    Serial.print(" dev ");
    if (captured >= 0)
    {
        Serial.print('+');
    }
    Serial.println(captured);

    has_captured    = true;
    captured_value  = value;
    captured_slot   = target.slot;
    unchanged_polls = 0;
    capture_now     = false;
    target_printed  = false;
    jigPlateauReset();

    if (target.kind == JigKind::Census)
    {
        last_census_valid = true;
        last_census_type  = target.type;
        last_census_piece = target.piece;
    }
}

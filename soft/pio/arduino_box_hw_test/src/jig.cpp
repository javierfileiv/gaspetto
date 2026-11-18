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
constexpr uint8_t JIG_TYPE_COUNT = 4;

static const BoxPieceId JIG_TYPES[JIG_TYPE_COUNT] = {
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
 * not the historical one: BAND_TO_PIECE in piece_decode.h carries the same */
static const char *const JIG_TYPE_HINTS[JIG_TYPE_COUNT] = {
    "green 4mm",  /* FORWARD: band 0, the most negative */
    "blue 4mm",   /* TURN_LEFT: band 4, the most positive */
    "yellow 1mm", /* TURN_RIGHT: band 1, the weak negative */
    "red 1mm",    /* LOOP_CALL: band 3, the weak positive */
};

/* Ascending class order the threshold table is built on: the row lists the
 * start of each band in this order, so a slot whose measured centres sort
 * differently would silently swap two pieces */
constexpr uint8_t JIG_CLASS_COUNT = 5;

/* Type index of each band in ascending order; 0xFF is the EMPTY band, which
 * has no piece and sits at the slot's own idle level. The bands ascend with
 * the raw reading, and this batch puts the greens at the bottom and the blues
 * at the top, so the order below is the reverse of the historical one and
 * matches BAND_TO_PIECE in piece_decode.h */
static const uint8_t JIG_CLASS_TYPE[JIG_CLASS_COUNT] = {0, 2, 0xFF, 3, 1};

/* Pieces per type, orientations per piece, slots to walk */
constexpr uint8_t JIG_PIECES_PER_TYPE = 5;

/* A type is still usable with fewer than five pieces: a broken one is
 * declared with 'n' and the plan stops asking for it. Three is the floor
 * where a median and a spread still mean something */
constexpr uint8_t JIG_MIN_PIECES = 3;

/* Poses the operator declared unavailable, remembered so the walk finishes
 * instead of asking for them forever */
constexpr uint8_t JIG_MAX_SKIPS    = 32;
constexpr uint8_t JIG_ORIENT_COUNT = 4;
constexpr uint8_t JIG_SLOT_COUNT   = CALIBRATION_SLOT_COUNT;

/* The census happens in one slot; the gain pass walks them all, the
 * reference one included so both ends of the ratio are measured at the same
 * moment */
constexpr uint8_t JIG_REFERENCE_SLOT = 0;

static_assert(JIG_SLOT_COUNT == GASPETTO_ADC_SLOTS, "jig slots must match the shared table");

/* Store keys that are not a type index */
constexpr uint8_t JIG_KEY_SLOT_IDLE = 0xFE; /* idle taken for the gain pass in a slot */

/* Detector thresholds, in counts. The weakest piece reads about 600 counts
 * away from idle, so 200 separates it from an empty slot with margin while
 * still catching a piece landing slightly off centre */
constexpr uint16_t JIG_PRESENT_DELTA = 200;

/* A plateau is six readings whose span stays inside 40 counts: one reading
 * is a fifteen sample mean already, and 40 counts is under 2 % of the
 * weakest piece, so the window can lock on a reading that drifts by a few
 * counts while the piece settles. A wild reading leaves the window after six
 * polls instead of poisoning it */
constexpr uint8_t JIG_STABLE_COUNT = 6;
constexpr uint16_t JIG_STABLE_SPAN = 40;

/* How far a reading may sit from the expected idle and still be accepted as
 * an idle level. A piece left in the slot reads hundreds of counts away, so
 * this refuses it instead of turning a piece into the slot's reference. The
 * slots idle up to 550 counts apart on this board, hence the margin */
constexpr uint16_t JIG_IDLE_PLAUSIBLE = 900;

/* Rejections of the idle before the firmware says what it is waiting for: a
 * silent loop would look like a frozen board */
constexpr uint8_t JIG_IDLE_REJECT_HINT = 10;

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
constexpr uint16_t JIG_REARM_DELTA = 30;
constexpr uint8_t JIG_STUCK_POLLS  = 8;

/* A pair of bands closer than this cannot survive the rail's wander and a
 * piece landing a little off: the slot is reported as tight and the sensor's
 * position becomes the lever */
constexpr int32_t JIG_TIGHT_GAP = 300;

/* A slot whose gain sits this far (per mille) from the median is reported:
 * too low and its bands squeeze against the empty one, too high and the
 * strongest piece approaches the sensor's ceiling */
constexpr int32_t JIG_GAIN_SPREAD_PERMILLE = 300;

/* Half-width added for the play in the slot, in per mille of the deviation.
 * The piece can land anywhere within that play, and the field falls with the
 * cube of the distance, so the error is proportional to the deviation, not a
 * fixed count. The 120 comes from the 588-count spread measured over four
 * orientations on a piece reading about 2700: that spread is the full width,
 * so half of it is about 11 %. Replace it with the 8-placement measurement
 * once it is done: four orientations at a fixed position, two positions */
constexpr int32_t JIG_PLAY_PERMILLE = 120;

/* Empty class half width: twice the rail wander measured on the bench, which
 * is the whole reason the empty band needs to be wider than a few counts */
constexpr uint16_t JIG_EMPTY_HALF = 100;

/* Rotation counts for a type when the four orientations move the reading by
 * more than this: below it the magnet is centred enough that the choice of
 * edge buys nothing, and asking four placements per piece would only waste
 * the operator's time */
constexpr uint16_t JIG_ROTATION_SIGNIFICANT = 50;

/* Pieces used to decide whether rotations matter for the rest of the type */
constexpr uint8_t JIG_ROTATION_PROBE_PIECES = 3;

/* A piece this far from its type's median is worth a look: at the magnet
 * sizes used here it is about the difference between two magnet lengths */
constexpr int32_t JIG_SUSPECT_PERMILLE = 200;

constexpr uint16_t JIG_ROUND_STEP = 5;

/* Room for the whole campaign plus redoes: repeating a placement replaces
 * the capture it supersedes */
constexpr uint16_t JIG_MAX_CAPTURES = 240;

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

static JigCapture captures[JIG_MAX_CAPTURES] = {};
static uint16_t capture_count                = 0;
static bool ref_known[JIG_SLOT_COUNT]        = {};
static uint16_t ref_value[JIG_SLOT_COUNT]    = {};

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
static int16_t window[JIG_STABLE_COUNT] = {};
static uint8_t window_count             = 0;
static uint8_t window_next              = 0;

/* Defined further down: the flash journal needs the store, and the store
 * reports to the journal after every change, so the two refer to each other */
static void jig_rebuild_refs();
static void jig_store_maybe_save();
static void jig_store_load();
static void jig_store_save();

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
constexpr int32_t JIG_IDLE_SUSPECT      = 200;
constexpr uint8_t JIG_MAX_IDLE_PER_SLOT = 12;

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

static bool jig_same_key(const JigCapture &capture, JigKind kind, uint8_t type, uint8_t piece,
                         uint8_t orient, uint8_t slot)
{
    return capture.kind == kind && capture.type == type && capture.piece == piece &&
           capture.orient == orient && capture.slot == slot;
}

static const JigCapture *jig_find(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient,
                                  uint8_t slot)
{
    /* Newest wins: a re-placed piece replaces the capture it supersedes */
    for (int32_t index = static_cast<int32_t>(capture_count) - 1; index >= 0; --index)
    {
        if (jig_same_key(captures[index], kind, type, piece, orient, slot))
        {
            return &captures[index];
        }
    }
    return nullptr;
}

static void jig_record(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot,
                       int16_t mean, uint16_t ref)
{
    if (capture_count >= JIG_MAX_CAPTURES)
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

    jig_store_maybe_save();
}

struct JigSkip
{
    JigKind kind;
    uint8_t type;
    uint8_t piece;
    uint8_t orient;
    uint8_t slot;
};

static JigSkip skips[JIG_MAX_SKIPS] = {};
static uint8_t skip_count           = 0;

static bool jig_is_skipped(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot)
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

static void jig_skip_add(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot)
{
    if (skip_count >= JIG_MAX_SKIPS)
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

    jig_store_maybe_save();
}

/* A capture counts as present when it was recorded or explicitly skipped */
static bool jig_missing(JigKind kind, uint8_t type, uint8_t piece, uint8_t orient, uint8_t slot)
{
    return jig_find(kind, type, piece, orient, slot) == nullptr &&
           !jig_is_skipped(kind, type, piece, orient, slot);
}

static uint16_t jig_ref_for(uint8_t slot)
{
    return ref_known[slot] ? ref_value[slot] : 0;
}

static int32_t jig_deviation(const JigCapture *capture)
{
    return static_cast<int32_t>(capture->mean) - static_cast<int32_t>(capture->ref);
}

static int32_t jig_gap(int32_t left, int32_t right)
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
constexpr uint8_t JIG_STORE_MAGIC_A     = 0x4A; /* 'J' */
constexpr uint8_t JIG_STORE_MAGIC_B     = 0x47; /* 'G' */
constexpr uint8_t JIG_STORE_VERSION     = 1;
constexpr uint16_t JIG_STORE_HEADER     = 8;
constexpr uint8_t JIG_STORE_RECORD_SIZE = 12;
constexpr uint16_t JIG_STORE_CAPACITY   = JIG_MAX_CAPTURES + JIG_MAX_SKIPS;
constexpr uint16_t JIG_STORE_TOTAL = JIG_STORE_HEADER + JIG_STORE_CAPACITY * JIG_STORE_RECORD_SIZE;
constexpr uint8_t JIG_STORE_KIND_SKIP = 3;
constexpr uint8_t JIG_SAVE_EVERY      = 4;

static_assert(JIG_STORE_TOTAL <= E2END + 1, "the journal must fit the emulated EEPROM");

static uint8_t unsaved    = 0;
static bool store_ready   = false;
static bool store_loading = false;

static void jig_store_write_u8(uint16_t position, uint8_t value)
{
    eeprom_buffered_write_byte(position, value);
}

static void jig_store_write_u16(uint16_t position, uint16_t value)
{
    jig_store_write_u8(position, static_cast<uint8_t>(value & 0xFFU));
    jig_store_write_u8(static_cast<uint16_t>(position + 1), static_cast<uint8_t>(value >> 8));
}

static uint8_t jig_store_read_u8(uint16_t position)
{
    return eeprom_buffered_read_byte(position);
}

static uint16_t jig_store_read_u16(uint16_t position)
{
    return static_cast<uint16_t>(
        jig_store_read_u8(position) |
        (static_cast<uint16_t>(jig_store_read_u8(static_cast<uint16_t>(position + 1))) << 8));
}

/* One record covers both a capture and a write-off: the kind byte says which,
 * and byte 5 carries the target kind a write-off applies to */
static void jig_store_write_record(uint16_t index, uint8_t kind, uint8_t type, uint8_t piece,
                                   uint8_t orient, uint8_t slot, uint8_t extra, int16_t mean,
                                   uint16_t ref)
{
    const uint16_t base = static_cast<uint16_t>(JIG_STORE_HEADER + index * JIG_STORE_RECORD_SIZE);

    jig_store_write_u8(base, kind);
    jig_store_write_u8(static_cast<uint16_t>(base + 1), type);
    jig_store_write_u8(static_cast<uint16_t>(base + 2), piece);
    jig_store_write_u8(static_cast<uint16_t>(base + 3), orient);
    jig_store_write_u8(static_cast<uint16_t>(base + 4), slot);
    jig_store_write_u8(static_cast<uint16_t>(base + 5), extra);
    jig_store_write_u16(static_cast<uint16_t>(base + 6), static_cast<uint16_t>(mean));
    jig_store_write_u16(static_cast<uint16_t>(base + 8), ref);
    jig_store_write_u16(static_cast<uint16_t>(base + 10), 0);
}

static void jig_store_save()
{
    if (store_loading)
    {
        return;
    }

    uint16_t index = 0;

    jig_store_write_u8(0, JIG_STORE_MAGIC_A);
    jig_store_write_u8(1, JIG_STORE_MAGIC_B);
    jig_store_write_u8(2, JIG_STORE_VERSION);
    jig_store_write_u16(3, static_cast<uint16_t>(capture_count + skip_count));

    for (uint16_t captureIndex = 0; captureIndex < capture_count && index < JIG_STORE_CAPACITY;
         ++captureIndex)
    {
        const JigCapture &capture = captures[captureIndex];
        jig_store_write_record(index++, static_cast<uint8_t>(capture.kind), capture.type,
                               capture.piece, capture.orient, capture.slot, 0, capture.mean,
                               capture.ref);
    }
    for (uint8_t skipIndex = 0; skipIndex < skip_count && index < JIG_STORE_CAPACITY; ++skipIndex)
    {
        const JigSkip &skip = skips[skipIndex];
        jig_store_write_record(index++, JIG_STORE_KIND_SKIP, skip.type, skip.piece, skip.orient,
                               skip.slot, static_cast<uint8_t>(skip.kind), 0, 0);
    }

    /* Blank the rest of the journal: a shorter save must not leave the tail
     * of a longer one behind, since the count alone would not catch it */
    for (; index < JIG_STORE_CAPACITY; ++index)
    {
        jig_store_write_record(index, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0);
    }

    eeprom_buffer_flush();

    unsaved     = 0;
    store_ready = true;

    Serial.print("   [flash ");
    Serial.print(capture_count);
    Serial.println(" captures]");
}

static void jig_store_maybe_save()
{
    if (store_loading)
    {
        return;
    }
    if (unsaved < 0xFF)
    {
        ++unsaved;
    }
    if (unsaved >= JIG_SAVE_EVERY)
    {
        jig_store_save();
    }
}

static void jig_store_load()
{
    if (store_ready)
    {
        return;
    }
    store_ready = true;

    eeprom_buffer_fill();

    if (jig_store_read_u8(0) != JIG_STORE_MAGIC_A || jig_store_read_u8(1) != JIG_STORE_MAGIC_B ||
        jig_store_read_u8(2) != JIG_STORE_VERSION)
    {
        return; /* blank page, or content from another firmware */
    }

    const uint16_t stored = jig_store_read_u16(3);
    if (stored == 0 || stored > JIG_STORE_CAPACITY)
    {
        return;
    }

    store_loading = true;
    for (uint16_t index = 0; index < stored; ++index)
    {
        const uint16_t base =
            static_cast<uint16_t>(JIG_STORE_HEADER + index * JIG_STORE_RECORD_SIZE);
        const uint8_t kind = jig_store_read_u8(base);

        if (kind == JIG_STORE_KIND_SKIP)
        {
            jig_skip_add(static_cast<JigKind>(jig_store_read_u8(static_cast<uint16_t>(base + 5))),
                         jig_store_read_u8(static_cast<uint16_t>(base + 1)),
                         jig_store_read_u8(static_cast<uint16_t>(base + 2)),
                         jig_store_read_u8(static_cast<uint16_t>(base + 3)),
                         jig_store_read_u8(static_cast<uint16_t>(base + 4)));
        }
        else if (kind <= 2)
        {
            jig_record(static_cast<JigKind>(kind),
                       jig_store_read_u8(static_cast<uint16_t>(base + 1)),
                       jig_store_read_u8(static_cast<uint16_t>(base + 2)),
                       jig_store_read_u8(static_cast<uint16_t>(base + 3)),
                       jig_store_read_u8(static_cast<uint16_t>(base + 4)),
                       static_cast<int16_t>(jig_store_read_u16(static_cast<uint16_t>(base + 6))),
                       jig_store_read_u16(static_cast<uint16_t>(base + 8)));
        }
    }
    store_loading = false;

    jig_rebuild_refs();

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
static bool jig_rotation_matters(uint8_t type)
{
    int32_t lowest  = 0;
    int32_t highest = 0;
    uint8_t pieces  = 0;

    for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE && pieces < JIG_ROTATION_PROBE_PIECES;
         ++piece)
    {
        int32_t pieceLow  = 0;
        int32_t pieceHigh = 0;
        bool any          = false;

        for (uint8_t orient = 0; orient < JIG_ORIENT_COUNT; ++orient)
        {
            const JigCapture *capture =
                jig_find(JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT);
            if (capture == nullptr)
            {
                continue;
            }
            const int32_t dev = jig_deviation(capture);
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

    return (highest - lowest) > static_cast<int32_t>(JIG_ROTATION_SIGNIFICANT);
}

static uint8_t jig_orientations_for(uint8_t type)
{
    return jig_rotation_matters(type) ? JIG_ORIENT_COUNT : 1;
}

/* The gain pass uses one piece per type, and always in orientation one: the
 * ratio only needs the same edge at both ends, and tying it to a fixed
 * orientation keeps the plan and the analysis looking up the same key. The
 * piece closest to the type's median is the least likely to sit at a band
 * edge and skew the scaling */
static uint8_t jig_ref_piece(uint8_t type)
{
    int32_t devs[JIG_PIECES_PER_TYPE]    = {};
    uint8_t used                         = 0;
    uint8_t pieceOf[JIG_PIECES_PER_TYPE] = {};

    for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE; ++piece)
    {
        const JigCapture *capture = jig_find(JigKind::Census, type, piece, 0, JIG_REFERENCE_SLOT);
        if (capture == nullptr)
        {
            continue;
        }
        devs[used]    = jig_deviation(capture);
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

static bool jig_next_target(JigTarget &target)
{
    /* Census: the reference slot's idle first, then five pieces per type in
     * every orientation still worth asking for. The idle is re-measured per
     * type so a census spread over several minutes never compares a reading
     * against an idle taken ten minutes earlier */
    for (uint8_t type = 0; type < JIG_TYPE_COUNT; ++type)
    {
        if (jig_missing(JigKind::Idle, type, 0, 0, JIG_REFERENCE_SLOT))
        {
            target = {JigKind::Idle, type, 0, 0, JIG_REFERENCE_SLOT};
            return true;
        }

        const uint8_t orientations = jig_orientations_for(type);
        for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE; ++piece)
        {
            for (uint8_t orient = 0; orient < orientations; ++orient)
            {
                if (jig_missing(JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT))
                {
                    target = {JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT};
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
    const uint8_t refPiece = jig_ref_piece(0);
    for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
    {
        if (jig_missing(JigKind::Idle, JIG_KEY_SLOT_IDLE, 0, 0, slot))
        {
            target = {JigKind::Idle, JIG_KEY_SLOT_IDLE, 0, 0, slot};
            return true;
        }
        if (jig_missing(JigKind::Gain, 0, refPiece, 0, slot))
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

static bool jig_read_slot(uint8_t slot, int16_t &mean)
{
    const AdsTestEntry &entry = ADS_TESTS[slot / 4];
    const uint8_t channel     = static_cast<uint8_t>(slot % 4);

    if (ads_read_single_ended_mean(&entry, channel, mean))
    {
        return true;
    }
    ads_recover_module(&entry);
    return ads_read_single_ended_mean(&entry, channel, mean);
}

static void jig_plateau_reset()
{
    window_count = 0;
    window_next  = 0;
}

static void jig_plateau_add(int16_t mean)
{
    window[window_next] = mean;
    window_next         = static_cast<uint8_t>((window_next + 1) % JIG_STABLE_COUNT);
    if (window_count < JIG_STABLE_COUNT)
    {
        ++window_count;
    }
}

static bool jig_plateau_stable()
{
    if (window_count < JIG_STABLE_COUNT)
    {
        return false;
    }

    int16_t lowest  = window[0];
    int16_t highest = window[0];
    for (uint8_t index = 1; index < JIG_STABLE_COUNT; ++index)
    {
        lowest  = window[index] < lowest ? window[index] : lowest;
        highest = window[index] > highest ? window[index] : highest;
    }
    return static_cast<uint16_t>(highest - lowest) <= JIG_STABLE_SPAN;
}

static int16_t jig_plateau_mean()
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

static const char *jig_orientation_hint(uint8_t orient)
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

static void jig_print_progress()
{
    uint16_t censusDone = 0;
    uint16_t censusPlan = 0;
    uint16_t gainDone   = 0;

    for (uint8_t type = 0; type < JIG_TYPE_COUNT; ++type)
    {
        const uint8_t orientations = jig_orientations_for(type);
        censusPlan += static_cast<uint16_t>(JIG_PIECES_PER_TYPE) * orientations;
        for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE; ++piece)
        {
            for (uint8_t orient = 0; orient < orientations; ++orient)
            {
                if (jig_find(JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT) != nullptr)
                {
                    ++censusDone;
                }
            }
        }
    }
    for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
    {
        if (jig_find(JigKind::Gain, 0, jig_ref_piece(0), 0, slot) != nullptr)
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
    Serial.print(JIG_SLOT_COUNT);
    Serial.println("   [q stop, d dump]");
}

static void jig_print_target(const JigTarget &target)
{
    Serial.println();

    switch (target.kind)
    {
    case JigKind::Idle:
        Serial.print("   [idle] slot ");
        Serial.print(static_cast<int>(target.slot) + 1);
        if (target.type != JIG_KEY_SLOT_IDLE)
        {
            Serial.print(" before ");
            Serial.print(JIG_TYPE_HINTS[target.type]);
        }
        Serial.println(": leave it empty, holding still");
        break;

    case JigKind::Census: {
        Serial.print("   [census] ");
        Serial.print(JIG_TYPE_HINTS[target.type]);
        Serial.print(", piece ");
        Serial.print(static_cast<int>(target.piece) + 1);
        Serial.print('/');
        Serial.print(JIG_PIECES_PER_TYPE);
        Serial.print(", orientation ");
        Serial.print(static_cast<int>(target.orient) + 1);
        Serial.print('/');
        Serial.print(jig_orientations_for(target.type));
        Serial.print(" (");
        Serial.print(jig_orientation_hint(target.orient));
        Serial.print(") in slot ");
        Serial.print(JIG_REFERENCE_SLOT + 1);
        Serial.println(": place it");
        break;
    }

    case JigKind::Gain: {
        Serial.print("   [gain] slot ");
        Serial.print(static_cast<int>(target.slot) + 1);
        Serial.print('/');
        Serial.print(JIG_SLOT_COUNT);
        Serial.print(": empty first, then ");
        Serial.print(JIG_TYPE_HINTS[0]);
        Serial.print(", piece ");
        Serial.print(static_cast<int>(target.piece) + 1);
        Serial.println(" (text readable)");
        break;
    }
    }

    jig_print_progress();
}

/* Which edge of the piece to paint, from the orientation that won. The
 * analysis names the winning pose as "text upside down"; that is not an
 * instruction anybody can act on, so the edge is named relative to the text
 * itself, which the operator can always read at a glance */
static const char *jig_mark_edge_hint(uint8_t orient)
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

static uint16_t jig_round5(int32_t value)
{
    if (value <= 0)
    {
        return 0;
    }
    if (value >= 65530)
    {
        return 65530;
    }
    return static_cast<uint16_t>(((value + (JIG_ROUND_STEP / 2)) / JIG_ROUND_STEP) *
                                 JIG_ROUND_STEP);
}

static void jig_sort(int32_t *values, uint8_t count)
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

static int32_t jig_median(const int32_t *sorted, uint8_t count)
{
    return count == 0 ? 0 : sorted[count / 2];
}

/* Deviation of one piece, taking the orientation closest to the type's
 * centre: an off-centre magnet reads differently a quarter turn apart, and
 * the edge the operator marks is exactly that best orientation */
static bool jig_piece_deviation(uint8_t type, uint8_t piece, int32_t centre, int32_t &deviation)
{
    bool any        = false;
    int32_t best    = 0;
    int32_t bestGap = 0;

    for (uint8_t orient = 0; orient < JIG_ORIENT_COUNT; ++orient)
    {
        const JigCapture *capture =
            jig_find(JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT);
        if (capture == nullptr)
        {
            continue;
        }
        const int32_t dev = jig_deviation(capture);
        const int32_t gap = jig_gap(dev, centre);
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

static void jig_analyse_pieces(JigTypeStats *stats)
{
    Serial.println("\n--- PIECE ANALYSIS (reference slot 1) ---");

    for (uint8_t type = 0; type < JIG_TYPE_COUNT; ++type)
    {
        int32_t all[JIG_PIECES_PER_TYPE * JIG_ORIENT_COUNT] = {};
        uint8_t allCount                                    = 0;

        for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE; ++piece)
        {
            for (uint8_t orient = 0; orient < JIG_ORIENT_COUNT; ++orient)
            {
                const JigCapture *capture =
                    jig_find(JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT);
                if (capture == nullptr)
                {
                    continue;
                }
                all[allCount++] = jig_deviation(capture);
            }
        }

        /* Pieces actually measured: a broken one is declared with 'n', so a
         * type can be complete with fewer than five and still give a median */
        uint8_t piecesSeen = 0;
        for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE; ++piece)
        {
            for (uint8_t orient = 0; orient < JIG_ORIENT_COUNT; ++orient)
            {
                if (jig_find(JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT) != nullptr)
                {
                    ++piecesSeen;
                    break;
                }
            }
        }

        stats[type].complete        = piecesSeen >= JIG_MIN_PIECES;
        stats[type].rotationMatters = jig_rotation_matters(type);

        Serial.print("  ");
        Serial.print(JIG_TYPE_HINTS[type]);
        Serial.print("  ");
        Serial.print(piecesSeen);
        Serial.print('/');
        Serial.print(JIG_PIECES_PER_TYPE);
        Serial.print(" pieces  as ");
        Serial.print(scan_label_name(static_cast<int>(JIG_TYPES[type])));

        if (!stats[type].complete)
        {
            Serial.println("  [INCOMPLETE]");
            continue;
        }

        jig_sort(all, allCount);
        const int32_t centre = jig_median(all, allCount);

        int32_t pieceDev[JIG_PIECES_PER_TYPE] = {};
        uint8_t used                          = 0;
        for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE; ++piece)
        {
            int32_t dev = 0;
            if (jig_piece_deviation(type, piece, centre, dev))
            {
                pieceDev[used++] = dev;
            }
        }

        jig_sort(pieceDev, used);
        const int32_t median = jig_median(pieceDev, used);

        int32_t maxDev = 0;
        for (uint8_t index = 0; index < used; ++index)
        {
            const int32_t gap = jig_gap(pieceDev[index], median);
            maxDev            = gap > maxDev ? gap : maxDev;
        }

        stats[type].median   = median;
        stats[type].maxDev   = maxDev;
        stats[type].refPiece = jig_ref_piece(type);

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

        const int32_t magnitude = jig_gap(median, 0);
        const int32_t suspect   = (magnitude * JIG_SUSPECT_PERMILLE) / 1000;

        for (uint8_t piece = 0; piece < JIG_PIECES_PER_TYPE; ++piece)
        {
            int32_t dev = 0;
            if (!jig_piece_deviation(type, piece, centre, dev))
            {
                continue;
            }
            const int32_t gap  = jig_gap(dev, median);
            uint8_t bestOrient = 0;
            int32_t bestGap    = 0;
            bool orientKnown   = false;
            int32_t orientLow  = 0;
            int32_t orientHigh = 0;

            for (uint8_t orient = 0; orient < JIG_ORIENT_COUNT; ++orient)
            {
                const JigCapture *capture =
                    jig_find(JigKind::Census, type, piece, orient, JIG_REFERENCE_SLOT);
                if (capture == nullptr)
                {
                    continue;
                }
                const int32_t orientDev = jig_deviation(capture);
                const int32_t orientGap = jig_gap(orientDev, dev);
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
            Serial.print(jig_orientation_hint(bestOrient));
            Serial.print(", ");
            Serial.print(jig_mark_edge_hint(bestOrient));
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
static bool jig_order_ok(const JigTypeStats *stats)
{
    for (uint8_t type = 0; type < JIG_TYPE_COUNT; ++type)
    {
        if (!stats[type].complete)
        {
            return false;
        }
    }

    return (stats[0].median < stats[2].median) && (stats[2].median < 0) && (stats[3].median > 0) &&
           (stats[1].median > stats[3].median);
}

static void jig_print_order(const JigTypeStats *stats)
{
    int32_t sorted[JIG_TYPE_COUNT] = {};
    uint8_t order[JIG_TYPE_COUNT]  = {};

    for (uint8_t type = 0; type < JIG_TYPE_COUNT; ++type)
    {
        sorted[type] = stats[type].median;
        order[type]  = type;
    }

    for (uint8_t i = 1; i < JIG_TYPE_COUNT; ++i)
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
    for (uint8_t i = 0; i < JIG_TYPE_COUNT; ++i)
    {
        Serial.print(JIG_TYPE_HINTS[order[i]]);
        Serial.print(" as ");
        Serial.print(scan_label_name(static_cast<int>(JIG_TYPES[order[i]])));
        Serial.print('(');
        if (sorted[i] >= 0)
        {
            Serial.print('+');
        }
        Serial.print(sorted[i]);
        Serial.print(')');
        Serial.print(i + 1 < JIG_TYPE_COUNT ? "  " : "");
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
static bool jig_gain_piece(uint8_t &piece)
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
static int32_t jig_gain_milli(uint8_t slot, uint8_t refPiece)
{
    const JigCapture *base = jig_find(JigKind::Gain, 0, refPiece, 0, JIG_REFERENCE_SLOT);
    if (base == nullptr)
    {
        return 0;
    }
    const int32_t baseDev = jig_deviation(base);
    if (baseDev == 0)
    {
        return 0;
    }

    const JigCapture *capture = jig_find(JigKind::Gain, 0, refPiece, 0, slot);
    if (capture == nullptr)
    {
        return 0;
    }
    return static_cast<int32_t>((static_cast<int64_t>(jig_deviation(capture)) * 1000) / baseDev);
}

static void jig_emit_rows(const JigTypeStats *stats)
{
    uint16_t rows[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS];
    uint16_t calibrated = 0;
    uint8_t refPiece    = stats[0].refPiece;

    /* Follow the gain pass rather than the census: a census redone after a
     * piece is re-glued names a different reference, and the gains already
     * measured stay valid because the ratio uses one piece at both ends */
    if (jig_gain_piece(refPiece))
    {
        Serial.print("\n   gain pass measured with ");
        Serial.print(JIG_TYPE_HINTS[0]);
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
    int32_t gains[JIG_SLOT_COUNT]       = {};
    int32_t sortedGains[JIG_SLOT_COUNT] = {};
    uint8_t gainCount                   = 0;
    for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
    {
        gains[slot] = jig_gain_milli(slot, refPiece);
        if (gains[slot] != 0)
        {
            sortedGains[gainCount++] = gains[slot];
        }
    }
    jig_sort(sortedGains, gainCount);
    const int32_t medianGain = jig_median(sortedGains, gainCount);

    bool flagged[JIG_SLOT_COUNT] = {};

    Serial.println("\n--- SLOT GAINS ---");

    for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
    {
        const uint16_t ref = jig_ref_for(slot);
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

        int32_t centre[JIG_CLASS_COUNT] = {};
        int32_t half[JIG_CLASS_COUNT]   = {};

        for (uint8_t cls = 0; cls < JIG_CLASS_COUNT; ++cls)
        {
            const uint8_t type = JIG_CLASS_TYPE[cls];
            if (type == 0xFF)
            {
                centre[cls] = ref;
                half[cls]   = JIG_EMPTY_HALF;
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
                (static_cast<int64_t>(jig_gap(stats[type].median, 0)) * JIG_PLAY_PERMILLE) / 1000);

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
        for (uint8_t cls = 1; cls < JIG_CLASS_COUNT; ++cls)
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
        const bool gainOff = (medianGain != 0) && (jig_gap(gain, medianGain) * 1000 >
                                                   medianGain * JIG_GAIN_SPREAD_PERMILLE);
        if (smallest < JIG_TIGHT_GAP || gainOff)
        {
            flagged[slot] = true;
        }

        uint16_t row[GASPETTO_ADC_COLS] = {};
        row[0]                          = jig_round5(centre[0] - half[0]);
        for (uint8_t cls = 1; cls < JIG_CLASS_COUNT; ++cls)
        {
            row[cls] =
                jig_round5(((centre[cls - 1] + half[cls - 1]) + (centre[cls] - half[cls])) / 2);
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
    Serial.println(JIG_SLOT_COUNT);

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

        for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
        {
            if (!flagged[slot])
            {
                continue;
            }
            anyFlagged = true;

            const uint16_t ref = jig_ref_for(slot);

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
            for (uint8_t cls = 0; cls < JIG_CLASS_COUNT; ++cls)
            {
                const uint8_t type = JIG_CLASS_TYPE[cls];
                int32_t centre     = static_cast<int32_t>(ref);
                int32_t half       = JIG_EMPTY_HALF;

                if (type != 0xFF && stats[type].complete)
                {
                    const int32_t play = static_cast<int32_t>(
                        (static_cast<int64_t>(jig_gap(stats[type].median, 0)) * JIG_PLAY_PERMILLE) /
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
                    Serial.print(scan_label_name(static_cast<int>(JIG_TYPES[type])));
                    Serial.print('/');
                    Serial.print(JIG_TYPE_HINTS[type]);
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
static void jig_print_idle_history()
{
    bool any = false;

    Serial.println("\n--- IDLE HISTORY (counts, deltas to the slot median) ---");

    for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
    {
        int32_t values[JIG_MAX_IDLE_PER_SLOT] = {};
        int32_t sorted[JIG_MAX_IDLE_PER_SLOT] = {};
        uint8_t count                         = 0;

        for (uint16_t index = 0; index < capture_count && count < JIG_MAX_IDLE_PER_SLOT; ++index)
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

        jig_sort(sorted, count);
        const int32_t median = jig_median(sorted, count);

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
            if (jig_gap(values[index], median) > JIG_IDLE_SUSPECT)
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

static void jig_analyse()
{
    JigTypeStats stats[JIG_TYPE_COUNT] = {};

    jig_print_idle_history();
    jig_analyse_pieces(stats);

    if (!jig_order_ok(stats))
    {
        jig_print_order(stats);
        analysis_done = true;
        jig_print_help();
        return;
    }

    jig_emit_rows(stats);

    analysis_done = true;
    Serial.println("\n   'j' reprints, 'd' dumps the captures, 'X' clears and restarts");
    jig_print_help();
}

/* ========================================== */
/* Shell entry points */
/* ========================================== */

/* The whole key list, printed by the shell's help and by '?' inside the
 * mode: one source, so a new key cannot show up in one and not the other */
void jig_print_help()
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

void jig_dump_captures()
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
            Serial.print(JIG_TYPE_HINTS[capture.type]);
            Serial.print(" piece ");
            Serial.print(static_cast<int>(capture.piece) + 1);
            Serial.print(" orient ");
            Serial.print(static_cast<int>(capture.orient) + 1);
            Serial.print(" dev ");
            Serial.print(jig_deviation(&capture));
            Serial.print(" ref ");
            Serial.println(capture.ref);
            break;
        case JigKind::Gain:
            Serial.print("  gain   slot ");
            Serial.print(static_cast<int>(capture.slot) + 1);
            Serial.print(" piece ");
            Serial.print(static_cast<int>(capture.piece) + 1);
            Serial.print(" dev ");
            Serial.print(jig_deviation(&capture));
            Serial.print(" ref ");
            Serial.println(capture.ref);
            break;
        }
    }
}

void jig_start_mode()
{
    if (test_mode != TestMode::Calibration)
    {
        Serial.println("jig mode works in calibration mode ('c' to switch)");
        return;
    }

    /* Whatever a reset left behind comes back before the plan is computed,
     * so the walk resumes on the pose it stopped at */
    jig_store_load();

    if (analysis_done)
    {
        jig_analyse();
        return;
    }

    jig_active     = true;
    target_printed = false;
    has_captured   = false;
    jig_plateau_reset();

    Serial.println("\n--- JIG MODE ---");
    if (capture_count > 0)
    {
        Serial.print("   resuming: ");
        Serial.print(capture_count);
        Serial.println(" captures in RAM");
    }
    Serial.println("   no keypress needed: place a piece and wait for the capture");
    jig_print_help();

    JigTarget target;
    if (!jig_next_target(target))
    {
        jig_analyse();
        jig_active = false;
    }
}

void jig_stop_mode()
{
    if (!jig_active)
    {
        return;
    }
    jig_active = false;
    jig_store_save();
    Serial.print("\n--- JIG MODE stopped, ");
    Serial.print(capture_count);
    Serial.println(" captures kept ---");
    jig_print_progress();
}

void jig_clear_captures()
{
    capture_count = 0;
    skip_count    = 0;
    analysis_done = false;
    has_captured  = false;
    for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
    {
        ref_known[slot] = false;
        ref_value[slot] = 0;
    }
    jig_store_save();
    Serial.println("   jig captures cleared");
}

void jig_forget_slot(uint8_t slot)
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
    jig_store_save();
}

/* Writing off the pose on screen: a piece that is broken or lost takes all
 * four of its orientations at once, a single pose only itself */
static void jig_skip_shown()
{
    if (!target_printed)
    {
        Serial.println("   no pose on screen to write off");
        return;
    }

    if (shown_target.kind == JigKind::Census)
    {
        for (uint8_t orient = 0; orient < JIG_ORIENT_COUNT; ++orient)
        {
            jig_skip_add(JigKind::Census, shown_target.type, shown_target.piece, orient,
                         shown_target.slot);
        }
        Serial.print("   piece ");
        Serial.print(static_cast<int>(shown_target.piece) + 1);
        Serial.print(" written off: ");
        Serial.println(JIG_TYPE_HINTS[shown_target.type]);
    }
    else
    {
        jig_skip_add(shown_target.kind, shown_target.type, shown_target.piece, shown_target.orient,
                     shown_target.slot);
        Serial.println("   pose written off");
    }

    target_printed = false;
    jig_plateau_reset();
}

/* The idle level of a slot is the last one recorded for it, so any change to
 * the store has to rebuild them: after an undo or a restart, the stale value
 * would be the one from the dropped capture */
static void jig_rebuild_refs()
{
    for (uint8_t slot = 0; slot < JIG_SLOT_COUNT; ++slot)
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
static void jig_restart_piece(uint8_t type, uint8_t piece)
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

    jig_rebuild_refs();

    Serial.print("   ");
    Serial.print(JIG_TYPE_HINTS[type]);
    Serial.print(" piece ");
    Serial.print(static_cast<int>(piece) + 1);
    Serial.print(" restarted, ");
    Serial.print(dropped);
    Serial.println(" poses dropped");

    analysis_done     = false;
    has_captured      = false;
    target_printed    = false;
    last_census_valid = false;
    jig_store_save();
    jig_plateau_reset();
}

/* Stepping back one capture: the pose comes back on screen and the operator
 * can place it again. Finer than restarting a whole piece when only the last
 * orientation was captured badly */
static void jig_undo_last()
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
        Serial.print(JIG_TYPE_HINTS[last.type]);
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
    jig_rebuild_refs();

    analysis_done  = false;
    has_captured   = false;
    target_printed = false;
    jig_store_save();
    jig_plateau_reset();
}

void jig_run_key(char key)
{
    switch (key)
    {
    case ' ':
        capture_now = true;
        Serial.println("   [capture on the next settled reading]");
        break;
    case 'd':
        jig_dump_captures();
        break;
    case 'n':
        if (last_census_valid)
        {
            jig_restart_piece(last_census_type, last_census_piece);
        }
        else if (target_printed && shown_target.kind == JigKind::Census)
        {
            jig_restart_piece(shown_target.type, shown_target.piece);
        }
        else
        {
            Serial.println("   no census piece to restart yet");
        }
        break;
    case 'b':
        jig_undo_last();
        break;
    case 'N':
        jig_skip_shown();
        break;
    case 'j':
    case 'q':
    case 'x':
        jig_stop_mode();
        break;
    case 'X':
        jig_clear_captures();
        jig_stop_mode();
        break;
    case 'k':
        Serial.println("   stop first ('q'), then type the slot number and k, e.g. 19k");
        break;
    case '?':
        jig_print_help();
        break;
    default:
        break;
    }
}

void jig_poll()
{
    JigTarget target;
    if (!jig_next_target(target))
    {
        if (!analysis_done)
        {
            jig_analyse();
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
        jig_print_target(target);
        shown_target    = target;
        target_printed  = true;
        idle_rejects    = 0;
        unchanged_polls = 0;
        jig_plateau_reset();
    }

    int16_t mean = 0;
    if (!jig_read_slot(target.slot, mean))
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
                                                           jig_ref_for(JIG_REFERENCE_SLOT);
        if (!capture_now && expected != 0 &&
            jig_gap(mean, expected) > static_cast<int32_t>(JIG_IDLE_PLAUSIBLE))
        {
            jig_plateau_reset();
            /* Silence would look like a hang: say what is refused and how to
             * override it, once every few seconds */
            if (idle_rejects < 0xFF)
            {
                ++idle_rejects;
            }
            if (idle_rejects == JIG_IDLE_REJECT_HINT)
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
        jig_plateau_add(mean);
        if (!jig_plateau_stable())
        {
            return;
        }

        const int16_t value = jig_plateau_mean();
        jig_record(JigKind::Idle, target.type, 0, 0, target.slot, value,
                   static_cast<uint16_t>(value));
        Serial.print("   idle measured: ");
        Serial.println(value);

        has_captured    = true;
        captured_value  = value;
        captured_slot   = target.slot;
        unchanged_polls = 0;
        capture_now     = false;
        target_printed  = false;
        jig_plateau_reset();
        return;
    }

    const uint16_t ref = jig_ref_for(target.slot);
    if (ref == 0)
    {
        Serial.println("   no idle reference for this slot, redo it with k<slot>");
        return;
    }

    if (!capture_now)
    {
        /* A piece has to be present... */
        if (jig_gap(mean, ref) < static_cast<int32_t>(JIG_PRESENT_DELTA))
        {
            jig_plateau_reset();
            return;
        }
        /* ...and the reading has to have moved since the last capture, since
         * a plateau that has not changed is the one already recorded. A
         * centred magnet never moves, so after a few refusals the capture is
         * taken anyway and the prompt says the value did not change */
        if (has_captured && captured_slot == target.slot &&
            jig_gap(mean, captured_value) < static_cast<int32_t>(JIG_REARM_DELTA))
        {
            if (unchanged_polls < 0xFF)
            {
                ++unchanged_polls;
            }
            if (unchanged_polls < JIG_STUCK_POLLS)
            {
                jig_plateau_reset();
                return;
            }
            if (unchanged_polls == JIG_STUCK_POLLS)
            {
                Serial.print("   reading did not move by ");
                Serial.print(JIG_REARM_DELTA);
                Serial.println(" counts: recording it as it stands");
            }
        }
        else
        {
            unchanged_polls = 0;
        }
    }

    jig_plateau_add(mean);
    if (!jig_plateau_stable())
    {
        return;
    }

    const int16_t value    = jig_plateau_mean();
    const int32_t captured = static_cast<int32_t>(value) - static_cast<int32_t>(ref);

    jig_record(target.kind, target.type, target.piece, target.orient, target.slot, value, ref);

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
    jig_plateau_reset();

    if (target.kind == JigKind::Census)
    {
        last_census_valid = true;
        last_census_type  = target.type;
        last_census_piece = target.piece;
    }
}

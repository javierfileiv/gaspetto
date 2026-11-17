#pragma once

#include <stdint.h>

/* Piece ADC thresholds, ranges in raw ADS1115 counts at GAIN_ONE.
 * Values derived from game-mode calibration campaign (n=8, 4 slots):
 *   TURN_LEFT:  mean=9863,  range=[8640..11038]
 *   BACKWARD:   mean=12371, range=[11903..12960]
 *   EMPTY:      mean=13311, range=[13137..13552]
 *   TURN_RIGHT: mean=14073, range=[13807..14325]
 *   FORWARD:    mean=15280, range=[14512..15956]
 *
 * Measured ascending order (by mean raw value):
 *   TURN_LEFT < BACKWARD < EMPTY < TURN_RIGHT < FORWARD < LOOP_CALL
 *
 * Thresholds placed at midpoints between adjacent classes.
 *
 * These global values are kept as a legacy fallback: the 12 calibrated
 * sensors idle anywhere between 12675 and 13616 counts, so no single
 * threshold can separate EMPTY from BACKWARD and TURN_RIGHT on every slot
 * at once. Decode reads the per-slot table below instead. */

#ifndef GASPETTO_ADC_THRESHOLD_TURN_LEFT_START
#define GASPETTO_ADC_THRESHOLD_TURN_LEFT_START 8640
#endif

#ifndef GASPETTO_ADC_THRESHOLD_BACKWARD_START
#define GASPETTO_ADC_THRESHOLD_BACKWARD_START 11470
#endif

#ifndef GASPETTO_ADC_THRESHOLD_EMPTY_START
#define GASPETTO_ADC_THRESHOLD_EMPTY_START 13048
#endif

#ifndef GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START
#define GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START 13679
#endif

#ifndef GASPETTO_ADC_THRESHOLD_FORWARD_START
#define GASPETTO_ADC_THRESHOLD_FORWARD_START 14418
#endif

#ifndef GASPETTO_ADC_THRESHOLD_LOOP_START
#define GASPETTO_ADC_THRESHOLD_LOOP_START 15957
#endif

#ifndef GASPETTO_ADC_THRESHOLD_LOOP_END
#define GASPETTO_ADC_THRESHOLD_LOOP_END 17551
#endif

#define GASPETTO_ADC_THRESHOLD_MAX 17551

/* Thresholds must be strictly increasing in measured ascending order */
static_assert(GASPETTO_ADC_THRESHOLD_TURN_LEFT_START > 0,
              "GASPETTO_ADC_THRESHOLD_TURN_LEFT_START must be > 0");
static_assert(GASPETTO_ADC_THRESHOLD_TURN_LEFT_START < GASPETTO_ADC_THRESHOLD_BACKWARD_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_BACKWARD_START < GASPETTO_ADC_THRESHOLD_EMPTY_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_EMPTY_START < GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START < GASPETTO_ADC_THRESHOLD_FORWARD_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_FORWARD_START < GASPETTO_ADC_THRESHOLD_LOOP_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_LOOP_START <= GASPETTO_ADC_THRESHOLD_LOOP_END,
              "GASPETTO_ADC_THRESHOLD_LOOP_START must be <= LOOP_END");
static_assert(GASPETTO_ADC_THRESHOLD_LOOP_END <= GASPETTO_ADC_THRESHOLD_MAX,
              "GASPETTO_ADC_THRESHOLD_LOOP_END must be <= GASPETTO_ADC_THRESHOLD_MAX");

/* Per-slot ADC thresholds, raw counts at GAIN_ONE, one row per slot and
 * columns in ascending raw order (GASPETTO_ADC_SLOT_COL_*). Each column is
 * the lower edge of a band and the decode walks the row until the first
 * boundary above the raw reading:
 *   col 0: band 0 start, below it the slot decodes INVALID
 *   col 1: end of band 0, start of band 1
 *   col 2: end of band 1, start of band 2 (the band carrying EMPTY)
 *   col 3: end of band 2, start of band 3
 *   col 4: end of band 3, start of band 4 (the highest band)
 *   col 5: sentinel pinned to 0xFFFF in every generated row, named
 *          COL_TO_REMOVE: no piece plays above the highest band so nothing
 *          reaches this boundary, and the whole column gets dropped
 *          as soon as the production migration moves GBox.cpp onto
 *          this per-slot table
 *
 * The band a piece name sits in is a convention, not physics: which pole a
 * magnet presents depends on how the piece was built, and the sensors cannot
 * tell a name. The constants below are therefore positional, and the bench
 * owns the band -> piece mapping in one place (kBandPieces in scan.cpp).
 * The column names keep the order the project used first, where TURN_LEFT
 * read the lowest: a batch mounted the other way round reverses the mapping,
 * not this header.
 *
 * An all-zero row means the slot is not calibrated: it decodes INVALID
 * rather than guessing, so a slot without thresholds cannot inject a false
 * piece. Loop-zone rows are zero because those slots are not soldered.
 * GASPETTO_ADC_SLOTS must match BOX_TOTAL_SLOTS. */
#define GASPETTO_ADC_SLOTS 20
#define GASPETTO_ADC_COLS 6

#define GASPETTO_ADC_SLOT_COL_TURN_LEFT 0
#define GASPETTO_ADC_SLOT_COL_LOOP_START 1
#define GASPETTO_ADC_SLOT_COL_EMPTY 2
#define GASPETTO_ADC_SLOT_COL_TURN_RIGHT 3
#define GASPETTO_ADC_SLOT_COL_FORWARD 4
#define GASPETTO_ADC_SLOT_COL_TO_REMOVE 5

/* --- BEGIN GENERATED PER-SLOT ROWS --- */
static const uint16_t GASPETTO_ADC_SLOT_THRESHOLDS[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS] = {
    /* slot 1 */ { 7500, 11870, 13025, 13610, 14545, 65535 },
    /* slot 2 */ { 7500, 11525, 12635, 13200, 14095, 65535 },
    /* slot 3 */ { 7500, 11100, 12625, 13360, 14595, 65535 },
    /* slot 4 */ { 7500, 11635, 12810, 13405, 14360, 65535 },
    /* slot 5 */ { 7500, 12150, 13150, 13675, 14480, 65535 },
    /* slot 6 */ { 7500, 12040, 13190, 13770, 14700, 65535 },
    /* slot 7 */ { 7500, 11695, 12620, 13110, 13860, 65535 },
    /* slot 8 */ { 7500, 10640, 12415, 13250, 14695, 65535 },
    /* slot 9 */ { 7500, 10780, 12525, 13355, 14775, 65535 },
    /* slot 10 */ { 7500, 11215, 12195, 12710, 13500, 65535 },
    /* slot 11 */ { 7500, 10605, 12470, 13345, 14865, 65535 },
    /* slot 12 */ { 7500, 11495, 12910, 13600, 14745, 65535 },
    /* slot 13 */ { 7500, 11890, 12775, 13255, 13970, 65535 },
    /* slot 14 */ { 7500, 11100, 12585, 13310, 14520, 65535 },
    /* slot 15 */ { 7500, 11975, 12945, 13460, 14245, 65535 },
    /* slot 16 */ { 7500, 11255, 12670, 13365, 14515, 65535 },
    /* slot 17 */ { 7500, 11230, 12735, 13465, 14685, 65535 },
    /* slot 18 */ { 7500, 11130, 12450, 13105, 14175, 65535 },
    /* slot 19 */ { 7500, 10575, 12525, 13435, 15020, 65535 },
    /* slot 20 */ { 7500, 10950, 12395, 13100, 14270, 65535 },
};
/* --- END GENERATED PER-SLOT ROWS --- */

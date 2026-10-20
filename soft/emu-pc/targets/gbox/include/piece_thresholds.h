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
 * columns in ascending piece order (GASPETTO_ADC_SLOT_COL_*). Each column
 * is the lower edge of a band and the decode walks the row until the first
 * boundary above the raw reading:
 *   col 0: TURN_LEFT band start, below it the slot decodes INVALID
 *   col 1: end of TURN_LEFT, start of LOOP_CALL (plays under the idle
 *          where BACKWARD used to live; renamed from the historical
 *          BACKWARD name to stop confusing the manual calibration)
 *   col 2: end of the loop band, start of EMPTY
 *   col 3: end of EMPTY, start of TURN_RIGHT
 *   col 4: end of TURN_RIGHT, start of FORWARD
 *   col 5: sentinel pinned to 0xFFFF in every generated row, named
 *          COL_TO_REMOVE: no piece plays above FORWARD so nothing
 *          reaches this boundary, and the whole column gets dropped
 *          as soon as the production migration moves GBox.cpp onto
 *          this per-slot table
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
    /* slot 1 */ { 10000, 11529, 13265, 13810, 15002, 65535 },
    /* slot 2 */ { 8500, 10900, 12600, 13400, 14813, 65535 },
    /* slot 3 */ { 9000, 11775, 12743, 13300, 15168, 65535 },
    /* slot 4 */ { 9300, 11300, 12790, 13638, 15150, 65535 },
    /* slot 5 */ { 8900, 11304, 13111, 14001, 15350, 65535 },
    /* slot 6 */ { 8365, 10558, 13192, 14184, 16034, 65535 },
    /* slot 7 */ { 9536, 11010, 12787, 13673, 15357, 65535 },
    /* slot 8 */ { 8512, 10384, 12656, 13353, 15417, 65535 },
    /* slot 9 */ { 9138, 10607, 12624, 13705, 15560, 65535 },
    /* slot 10 */ { 8210, 10094, 12161, 13011, 14118, 65535 },
    /* slot 11 */ { 8669, 10665, 12894, 13767, 15138, 65535 },
    /* slot 12 */ { 8730, 10903, 13203, 14149, 16064, 65535 },
    /* slot 13 */ { 0, 0, 0, 0, 0, 0 },
    /* slot 14 */ { 0, 0, 0, 0, 0, 0 },
    /* slot 15 */ { 0, 0, 0, 0, 0, 0 },
    /* slot 16 */ { 0, 0, 0, 0, 0, 0 },
    /* slot 17 */ { 0, 0, 0, 0, 0, 0 },
    /* slot 18 */ { 0, 0, 0, 0, 0, 0 },
    /* slot 19 */ { 0, 0, 0, 0, 0, 0 },
    /* slot 20 */ { 0, 0, 0, 0, 0, 0 },
};
/* --- END GENERATED PER-SLOT ROWS --- */

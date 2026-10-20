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
 * columns in ascending piece order (GASPETTO_ADC_SLOT_COL_*). An all-zero
 * row means the slot is not calibrated: it decodes INVALID rather than
 * guessing, so a slot without thresholds cannot inject a false piece.
 * Main-zone rows are seeded with the global values above until a per-slot
 * campaign is pasted; loop-zone rows are zero because those slots are not
 * soldered. LOOP_START carries the global placeholder in every row until
 * the orange piece exists. GASPETTO_ADC_SLOTS must match BOX_TOTAL_SLOTS. */
#define GASPETTO_ADC_SLOTS 20
#define GASPETTO_ADC_COLS 6

#define GASPETTO_ADC_SLOT_COL_TURN_LEFT 0
#define GASPETTO_ADC_SLOT_COL_BACKWARD 1
#define GASPETTO_ADC_SLOT_COL_EMPTY 2
#define GASPETTO_ADC_SLOT_COL_TURN_RIGHT 3
#define GASPETTO_ADC_SLOT_COL_FORWARD 4
#define GASPETTO_ADC_SLOT_COL_LOOP_START 5

/* --- BEGIN GENERATED PER-SLOT ROWS --- */
static const uint16_t GASPETTO_ADC_SLOT_THRESHOLDS[GASPETTO_ADC_SLOTS][GASPETTO_ADC_COLS] = {
    /* slot 1 */ { 10848, 11976, 13289, 13848, 14560, 15957 },
    /* slot 2 */ { 8730, 10374, 12535, 13567, 14678, 15957 },
    /* slot 3 */ { 9520, 10816, 12624, 13542, 14624, 15957 },
    /* slot 4 */ { 9264, 10832, 12817, 13657, 14680, 15957 },
    /* slot 5 */ { 9984, 11272, 13048, 13944, 15174, 15957 },
    /* slot 6 */ { 9696, 11073, 13034, 14208, 15575, 15957 },
    /* slot 7 */ { 9856, 11088, 12712, 13517, 14648, 15957 },
    /* slot 8 */ { 9216, 10504, 12431, 13551, 15160, 15957 },
    /* slot 9 */ { 10260, 11342, 12794, 13612, 15288, 15957 },
    /* slot 10 */ { 9267, 10490, 12198, 13227, 13798, 15957 },
    /* slot 11 */ { 10123, 11121, 12620, 13644, 14597, 15957 },
    /* slot 12 */ { 8763, 10670, 13032, 13828, 15074, 15957 },
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

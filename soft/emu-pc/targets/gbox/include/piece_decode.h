#pragma once

#include "CommandPacket.h"
#include "piece_thresholds.h"

#include <stdint.h>

/* Which piece each band carries. The bands ascend with the raw reading, so
 * this array is the single place where a name is bound to a physical family.
 * This batch was mounted with the opposite pole throughout, which puts the
 * greens at the bottom and the blues at the top; the operator asked for the
 * greens to be FORWARD and the reds LOOP_CALL, so the mapping follows the
 * measurement instead of asking twenty magnets to be flipped. Rebuilding the
 * pieces the other way round flips this array, nothing else.
 * BACKWARD has no band: no physical piece carries it, and the command stays
 * available for a program built another way */
static constexpr BoxPieceId BAND_TO_PIECE[5] = {
    BoxPieceId::FORWARD, /* band 0: green 4 mm, the most negative */
    BoxPieceId::TURN_RIGHT, /* band 1: yellow 2 mm */
    BoxPieceId::EMPTY, /* band 2: the slot's own idle level */
    BoxPieceId::LOOP_CALL, /* band 3: red 2 mm */
    BoxPieceId::TURN_LEFT, /* band 4: blue 4 mm, the most positive */
};

/* Column that opens each band in a per-slot row. The sentinel closes the last
 * one, so band b spans [BAND_START_COLUMNS[b], BAND_START_COLUMNS[b + 1]) */
static constexpr uint8_t BAND_START_COLUMNS[6] = {
    GASPETTO_ADC_SLOT_COL_TURN_LEFT, GASPETTO_ADC_SLOT_COL_LOOP_START,
    GASPETTO_ADC_SLOT_COL_EMPTY,     GASPETTO_ADC_SLOT_COL_TURN_RIGHT,
    GASPETTO_ADC_SLOT_COL_FORWARD,   GASPETTO_ADC_SLOT_COL_TO_REMOVE,
};

/* Decode one raw reading with this slot's row of the per-slot threshold
 * table. The columns are ascending piece starts, so the first column below
 * which nothing matches means the slot is not calibrated: it decodes INVALID
 * rather than guessing with a global fallback. This is the only decode in
 * the project: the bench monitors, the bench scan and the production box all
 * go through it, so a label printed on the bench and a piece sent over the
 * air can never come from two different walks */
inline BoxPieceId piece_decode(uint8_t slot, int16_t raw)
{
    if (slot >= GASPETTO_ADC_SLOTS || raw < 0) {
        return BoxPieceId::INVALID;
    }

    const uint16_t *row = GASPETTO_ADC_SLOT_THRESHOLDS[slot];

    if ((row[GASPETTO_ADC_SLOT_COL_TURN_LEFT] == 0U) ||
        (raw < row[GASPETTO_ADC_SLOT_COL_TURN_LEFT])) {
        return BoxPieceId::INVALID;
    }

    for (uint8_t band = 0; band < 5; ++band) {
        if (raw < row[BAND_START_COLUMNS[band + 1]]) {
            return BAND_TO_PIECE[band];
        }
    }

    return BoxPieceId::INVALID;
}

/* Reverse of piece_decode, used by the emulation to fabricate a reading that
 * decodes back to the piece: the middle of the band carrying it. A piece with
 * no band, or an uncalibrated slot, returns 0, which decodes INVALID */
inline uint16_t piece_raw_for_slot(uint8_t slot, BoxPieceId piece)
{
    if (slot >= GASPETTO_ADC_SLOTS) {
        return 0;
    }

    const uint16_t *row = GASPETTO_ADC_SLOT_THRESHOLDS[slot];

    if (row[GASPETTO_ADC_SLOT_COL_TURN_LEFT] == 0U) {
        return 0;
    }

    for (uint8_t band = 0; band < 5; ++band) {
        if (BAND_TO_PIECE[band] != piece) {
            continue;
        }

        const uint32_t low = row[BAND_START_COLUMNS[band]];
        const uint32_t high = row[BAND_START_COLUMNS[band + 1]];
        uint32_t middle = low + ((high - low) / 2U);

        /* Readings travel as int16_t, and the top band's sentinel is 0xFFFF:
         * the clamp stays inside the band and keeps the value positive */
        if (middle > 32767U) {
            middle = 32767U;
        }

        return static_cast<uint16_t>(middle);
    }

    return 0;
}

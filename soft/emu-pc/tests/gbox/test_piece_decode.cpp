#include "piece_decode.h"

#include <cstdint>
#include <gtest/gtest.h>

namespace
{
/* Slot 1's row, the edges the expectations below are written against:
 * { 7500, 11870, 13025, 13610, 14545, 65535 } */
constexpr uint8_t kSlot = 0;
} // namespace

TEST(PieceDecodeTest, EachBandDecodesToItsPiece)
{
    EXPECT_EQ(piece_decode(kSlot, 8000), BoxPieceId::FORWARD);
    EXPECT_EQ(piece_decode(kSlot, 12000), BoxPieceId::TURN_RIGHT);
    EXPECT_EQ(piece_decode(kSlot, 13200), BoxPieceId::EMPTY);
    EXPECT_EQ(piece_decode(kSlot, 13800), BoxPieceId::LOOP_CALL);
    EXPECT_EQ(piece_decode(kSlot, 15000), BoxPieceId::TURN_LEFT);
}

TEST(PieceDecodeTest, AThresholdBelongsToTheUpperBand)
{
    EXPECT_EQ(piece_decode(kSlot, 11869), BoxPieceId::FORWARD);
    EXPECT_EQ(piece_decode(kSlot, 11870), BoxPieceId::TURN_RIGHT);
}

TEST(PieceDecodeTest, ReadingsOutsideTheRowAreInvalid)
{
    EXPECT_EQ(piece_decode(kSlot, 7499), BoxPieceId::INVALID);
    EXPECT_EQ(piece_decode(kSlot, -1), BoxPieceId::INVALID);
    EXPECT_EQ(piece_decode(kSlot, 65535), BoxPieceId::INVALID);
    EXPECT_EQ(piece_decode(GASPETTO_ADC_SLOTS, 12000), BoxPieceId::INVALID);
}

TEST(PieceDecodeTest, TheReverseFabricatesAReadingInTheSameBand)
{
    const BoxPieceId pieces[5] = { BoxPieceId::FORWARD, BoxPieceId::TURN_RIGHT, BoxPieceId::EMPTY,
                                   BoxPieceId::LOOP_CALL, BoxPieceId::TURN_LEFT };

    for (BoxPieceId piece : pieces) {
        const uint16_t raw = piece_raw_for_slot(kSlot, piece);

        EXPECT_NE(raw, 0U);
        EXPECT_EQ(piece_decode(kSlot, static_cast<int16_t>(raw)), piece);
    }
}

TEST(PieceDecodeTest, APieceWithoutABandHasNoReading)
{
    EXPECT_EQ(piece_raw_for_slot(kSlot, BoxPieceId::BACKWARD), 0U);
    EXPECT_EQ(piece_raw_for_slot(GASPETTO_ADC_SLOTS, BoxPieceId::FORWARD), 0U);
}

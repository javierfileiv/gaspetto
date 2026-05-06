#include "Adafruit_ADS1X15.h"
#include "GaspettoBox.h"
#include "Wire.h"
#include "board_presets.h"

#include <gtest/gtest.h>

TEST(GaspettoBoxProgramBuilderTest, ExpandsLoopSequenceIntoPayload)
{
    BoxBoardPieces board = gaspetto_box_test::emptyBoardPieces();
    CommandPacket packet{};
    bool isEmpty = false;

    board[0] = BoxPieceId::FORWARD;
    board[1] = BoxPieceId::TURN_RIGHT;
    board[2] = BoxPieceId::LOOP_CALL;
    board[3] = BoxPieceId::STOP;

    board[14] = BoxPieceId::TURN_LEFT;
    board[15] = BoxPieceId::FORWARD;
    board[16] = BoxPieceId::STOP;

    const bool ok = GaspettoBox::buildProgramFromPieces(board, packet, isEmpty);

    ASSERT_TRUE(ok);
    ASSERT_FALSE(isEmpty);
    ASSERT_EQ(packet.count, 6);
    EXPECT_EQ(packet.commands[0], static_cast<uint8_t>(CommandId::MOTOR_FORWARD));
    EXPECT_EQ(packet.commands[1], static_cast<uint8_t>(CommandId::MOTOR_TURN_RIGHT));
    EXPECT_EQ(packet.commands[2], static_cast<uint8_t>(CommandId::MOTOR_TURN_LEFT));
    EXPECT_EQ(packet.commands[3], static_cast<uint8_t>(CommandId::MOTOR_FORWARD));
    EXPECT_EQ(packet.commands[4], static_cast<uint8_t>(CommandId::MOTOR_STOP));
    EXPECT_EQ(packet.commands[5], static_cast<uint8_t>(CommandId::MOTOR_STOP));
}

TEST(GaspettoBoxProgramBuilderTest, RejectsEmptyBoard)
{
    BoxBoardPieces board = gaspetto_box_test::emptyBoardPieces();
    CommandPacket packet{};
    bool isEmpty = false;

    const bool ok = GaspettoBox::buildProgramFromPieces(board, packet, isEmpty);

    EXPECT_FALSE(ok);
    EXPECT_TRUE(isEmpty);
    EXPECT_EQ(packet.count, 0);
}

TEST(GaspettoBoxProgramBuilderTest, RejectsOverflowingLoopExpansion)
{
    BoxBoardPieces board = gaspetto_box_test::kOverflowBoardPieces;
    CommandPacket packet{};
    bool isEmpty = false;

    const bool ok = GaspettoBox::buildProgramFromPieces(board, packet, isEmpty);

    EXPECT_FALSE(ok);
    EXPECT_FALSE(isEmpty);
    EXPECT_EQ(packet.count, BOX_MAX_PAYLOAD_COMMANDS);
}

TEST(GaspettoBoxProgramBuilderTest, RejectsLoopTokenInsideLoopArea)
{
    BoxBoardPieces board = gaspetto_box_test::emptyBoardPieces();
    CommandPacket packet{};
    bool isEmpty = false;

    board[0] = BoxPieceId::LOOP_CALL;
    board[14] = BoxPieceId::LOOP_CALL;

    const bool ok = GaspettoBox::buildProgramFromPieces(board, packet, isEmpty);

    EXPECT_FALSE(ok);
    EXPECT_FALSE(isEmpty);
}

TEST(GaspettoBoxProgramBuilderTest, RejectsInvalidTokenInMainArea)
{
    BoxBoardPieces board = gaspetto_box_test::emptyBoardPieces();
    CommandPacket packet{};
    bool isEmpty = false;

    board[0] = BoxPieceId::INVALID;

    const bool ok = GaspettoBox::buildProgramFromPieces(board, packet, isEmpty);

    EXPECT_FALSE(ok);
    EXPECT_FALSE(isEmpty);
}

TEST(GaspettoBoxProgramBuilderTest, RejectsUnknownMainTokenValue)
{
    BoxBoardPieces board = gaspetto_box_test::emptyBoardPieces();
    CommandPacket packet{};
    bool isEmpty = false;

    board[0] = static_cast<BoxPieceId>(0xFF);

    const bool ok = GaspettoBox::buildProgramFromPieces(board, packet, isEmpty);

    EXPECT_FALSE(ok);
    EXPECT_FALSE(isEmpty);
}

TEST(GaspettoBoxProgramBuilderTest, RejectsOverflowWhenAppendingRegularMainPiece)
{
    BoxBoardPieces board = gaspetto_box_test::emptyBoardPieces();
    CommandPacket packet{};
    bool isEmpty = false;

    /* 5 loop calls with 6 loop commands each => 30 commands. */
    board[0] = BoxPieceId::LOOP_CALL;
    board[1] = BoxPieceId::LOOP_CALL;
    board[2] = BoxPieceId::LOOP_CALL;
    board[3] = BoxPieceId::LOOP_CALL;
    board[4] = BoxPieceId::LOOP_CALL;

    /* One normal append reaches 31, next normal append must fail. */
    board[5] = BoxPieceId::FORWARD;
    board[6] = BoxPieceId::STOP;

    board[14] = BoxPieceId::FORWARD;
    board[15] = BoxPieceId::BACKWARD;
    board[16] = BoxPieceId::TURN_LEFT;
    board[17] = BoxPieceId::TURN_RIGHT;
    board[18] = BoxPieceId::STOP;
    board[19] = BoxPieceId::FORWARD;

    const bool ok = GaspettoBox::buildProgramFromPieces(board, packet, isEmpty);

    EXPECT_FALSE(ok);
    EXPECT_FALSE(isEmpty);
    EXPECT_EQ(packet.count, BOX_MAX_PAYLOAD_COMMANDS);
}

TEST(GaspettoBoxProgramBuilderTest, MapsSlotsToExpectedAdsBusesAndAddresses)
{
    AdsRouteInfo route{};

    for (std::size_t slot = 0; slot < 16; ++slot) {
        ASSERT_TRUE(GaspettoBox::routeForSlot(slot, route));
        EXPECT_FALSE(route.usesI2c3);
        EXPECT_EQ(route.address, static_cast<uint8_t>(0x48 + (slot / 4)));
        EXPECT_EQ(route.channel, static_cast<uint8_t>(slot % 4));
    }

    for (std::size_t slot = 16; slot < BOX_TOTAL_SLOTS; ++slot) {
        ASSERT_TRUE(GaspettoBox::routeForSlot(slot, route));
        EXPECT_TRUE(route.usesI2c3);
        EXPECT_EQ(route.address, 0x4A);
        EXPECT_EQ(route.channel, static_cast<uint8_t>(slot - 16));
    }

    EXPECT_FALSE(GaspettoBox::routeForSlot(BOX_TOTAL_SLOTS, route));
}

TEST(AdafruitAdsEmulationTest, AsyncReadReturnsMuxSelectedI2c3RawValue)
{
    Adafruit_ADS1115::resetPattern();
    Adafruit_ADS1115::setRawValue(16, 1234);
    Adafruit_ADS1115::setRawValue(19, 3210);

    TwoWire i2c3;
    Adafruit_ADS1115 ads;

    ASSERT_TRUE(ads.begin(0x4A, &i2c3));

    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
    EXPECT_TRUE(ads.conversionComplete());
    EXPECT_EQ(1234, ads.getLastConversionResults());

    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_3, false);
    EXPECT_TRUE(ads.conversionComplete());
    EXPECT_EQ(3210, ads.getLastConversionResults());

    Adafruit_ADS1115::resetPattern();
}

TEST(AdafruitAdsEmulationTest, RejectsUnexpectedI2c3Address)
{
    TwoWire i2c3;
    Adafruit_ADS1115 ads;

    EXPECT_FALSE(ads.begin(0x48, &i2c3));
}

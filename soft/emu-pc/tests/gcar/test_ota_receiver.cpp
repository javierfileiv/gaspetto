#include "Crc32.h"
#include "FlashLayout.h"
#include "FlashWriter.h"
#include "OtaProtocol.h"
#include "OtaReceiver.h"
#include "PacketTag.h"
#include "RadioController.h"
#include "RecordStore.h"
#include "config_radio.h"
#include "mock_Arduino.h"
#include "mock_RF24.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <vector>

using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;

namespace
{
/* Three packets cover the word loop, the sequence accounting and the CRC
 * walk without a long image */
constexpr uint16_t IMAGE_PACKETS = 3;
constexpr uint32_t IMAGE_LENGTH = IMAGE_PACKETS * kOtaPayload;
constexpr uint8_t WRONG_VERSION = static_cast<uint8_t>(kWireProtocolVersion) + 1;

class OtaReceiverTest : public ::testing::Test {
protected:
    /* The answer retry loop calls delay() between attempts, and the emulated
     * Arduino routes it through this mock */
    NiceMock<MockArduino> arduino;
    NiceMock<MockRF24> radio;
    EventQueue queue;
    RadioController controller{ radio, &queue, gbox_pipe_name, gcar_pipe_name };
    OtaReceiver receiver{ controller };
    std::vector<OtaPacket> answers;

    void SetUp() override
    {
        std::memset(gStagingSector, 0xFF, sizeof(gStagingSector));
        std::memset(gRecordSector, 0xFF, sizeof(gRecordSector));
        flashFailureForTest() = false;
        answers.clear();
        ON_CALL(radio, _write(_, _))
                .WillByDefault(Invoke([this](const void *buffer, uint8_t length) {
                    if (length == sizeof(OtaPacket)) {
                        OtaPacket packet{};
                        std::memcpy(&packet, buffer, sizeof(packet));
                        answers.push_back(packet);
                    }
                    return true;
                }));
    }

    void TearDown() override
    {
        flashFailureForTest() = false;
    }

    static std::array<uint8_t, IMAGE_LENGTH> makeImage()
    {
        std::array<uint8_t, IMAGE_LENGTH> image{};
        for (std::size_t index = 0; index < image.size(); ++index) {
            image[index] = static_cast<uint8_t>((index * 37U + 11U) & 0xFFU);
        }
        return image;
    }

    static void putLe32(uint8_t *at, uint32_t value)
    {
        at[0] = static_cast<uint8_t>(value & 0xFFU);
        at[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
        at[2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
        at[3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
    }

    static OtaPacket makeBegin(uint32_t length, uint32_t crc,
                               uint8_t version = kWireProtocolVersion)
    {
        OtaPacket packet = {};

        packet.tag = static_cast<uint8_t>(PacketTag::Ota);
        packet.kind = static_cast<uint8_t>(OtaKind::Begin);
        packet.data[0] = version;
        putLe32(&packet.data[2], length);
        putLe32(&packet.data[6], crc);
        return packet;
    }

    static OtaPacket makeData(uint16_t seq, const std::array<uint8_t, IMAGE_LENGTH> &image)
    {
        OtaPacket packet = {};

        packet.tag = static_cast<uint8_t>(PacketTag::Ota);
        packet.kind = static_cast<uint8_t>(OtaKind::Data);
        packet.seq = seq;
        std::memcpy(packet.data, &image[static_cast<std::size_t>(seq) * kOtaPayload], kOtaPayload);
        return packet;
    }

    static OtaPacket makeEnd(uint16_t seq)
    {
        OtaPacket packet = {};

        packet.tag = static_cast<uint8_t>(PacketTag::Ota);
        packet.kind = static_cast<uint8_t>(OtaKind::End);
        packet.seq = seq;
        return packet;
    }

    void sendBegin(const std::array<uint8_t, IMAGE_LENGTH> &image, uint32_t crc = 0)
    {
        const uint32_t value = crc != 0 ? crc : crc32(image.data(), IMAGE_LENGTH);

        receiver.handle(makeBegin(IMAGE_LENGTH, value));
    }

    void sendAllData(const std::array<uint8_t, IMAGE_LENGTH> &image)
    {
        for (uint16_t seq = 0; seq < IMAGE_PACKETS; ++seq) {
            receiver.handle(makeData(seq, image));
        }
    }
};
} /* namespace */

TEST_F(OtaReceiverTest, BeginErasesTheStagingSectorAndAcknowledges)
{
    const auto image = makeImage();
    std::memset(gStagingSector, 0xAA, sizeof(gStagingSector));

    sendBegin(image);

    ASSERT_EQ(1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Ack), answers[0].kind);
    EXPECT_EQ(kOtaBeginAck, answers[0].seq);
    EXPECT_EQ(0xFF, gStagingSector[0]);
    EXPECT_EQ(0xFF, gStagingSector[IMAGE_LENGTH]);
}

TEST_F(OtaReceiverTest, BeginRefusesAnotherProtocolVersion)
{
    const auto image = makeImage();

    receiver.handle(makeBegin(IMAGE_LENGTH, crc32(image.data(), IMAGE_LENGTH), WRONG_VERSION));

    ASSERT_EQ(1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers[0].kind);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::ProtocolMismatch), answers[0].data[0]);
}

TEST_F(OtaReceiverTest, BeginRefusesAnEmptyImage)
{
    receiver.handle(makeBegin(0, 0));

    ASSERT_EQ(1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::TooLarge), answers[0].data[0]);
}

TEST_F(OtaReceiverTest, BeginRefusesAnImageLargerThanStaging)
{
    receiver.handle(makeBegin(kStagingCapacity + 1U, 0));

    ASSERT_EQ(1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::TooLarge), answers[0].data[0]);
}

TEST_F(OtaReceiverTest, BeginReportsAnEraseFailure)
{
    const auto image = makeImage();
    flashFailureForTest() = true;

    sendBegin(image);

    ASSERT_EQ(1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers[0].kind);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::FlashFailure), answers[0].data[0]);
}

TEST_F(OtaReceiverTest, DataBeforeABeginReportsAGap)
{
    const auto image = makeImage();

    receiver.handle(makeData(0, image));

    ASSERT_EQ(1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers[0].kind);
    EXPECT_EQ(0, answers[0].seq);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::SequenceGap), answers[0].data[0]);
    EXPECT_EQ(0xFF, gStagingSector[0]);
}

TEST_F(OtaReceiverTest, RepeatedDataIsAcknowledgedAgain)
{
    const auto image = makeImage();
    sendBegin(image);

    receiver.handle(makeData(0, image));
    receiver.handle(makeData(0, image));

    /* The sender retries because an answer was lost: acknowledge it again. */
    ASSERT_EQ(3U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Ack), answers[2].kind);
    EXPECT_EQ(0, answers[2].seq);
}

TEST_F(OtaReceiverTest, DataWithAGapReportsTheExpectedSequence)
{
    const auto image = makeImage();
    sendBegin(image);

    receiver.handle(makeData(1, image));

    ASSERT_EQ(2U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers[1].kind);
    EXPECT_EQ(0, answers[1].seq);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::SequenceGap), answers[1].data[0]);

    /* The session survives the gap: the missing packet can still arrive. */
    receiver.handle(makeData(0, image));
    ASSERT_EQ(3U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Ack), answers[2].kind);
}

TEST_F(OtaReceiverTest, DataReportsAWriteFailure)
{
    const auto image = makeImage();
    sendBegin(image);
    flashFailureForTest() = true;

    receiver.handle(makeData(0, image));

    ASSERT_EQ(2U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers[1].kind);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::FlashFailure), answers[1].data[0]);
}

TEST_F(OtaReceiverTest, EndWithoutABeginReportsAGap)
{
    receiver.handle(makeEnd(0));

    ASSERT_EQ(1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers[0].kind);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::SequenceGap), answers[0].data[0]);
}

TEST_F(OtaReceiverTest, EndWithAMissingPacketReportsTheGap)
{
    const auto image = makeImage();
    receiver.handle(makeBegin(2U * kOtaPayload, crc32(image.data(), 2U * kOtaPayload)));
    receiver.handle(makeData(0, image));

    receiver.handle(makeEnd(0));

    /* Begin and one Data were acknowledged, the End answer comes last. */
    ASSERT_EQ(1U + 1U + 1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers.back().kind);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::SequenceGap), answers.back().data[0]);
}

TEST_F(OtaReceiverTest, EndWithAWrongCrcIsRefused)
{
    const auto image = makeImage();
    const uint32_t wrongCrc = crc32(image.data(), IMAGE_LENGTH) ^ 1U;

    sendBegin(image, wrongCrc);
    sendAllData(image);

    receiver.handle(makeEnd(IMAGE_PACKETS - 1));

    ASSERT_EQ(1U + IMAGE_PACKETS + 1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers.back().kind);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::CrcMismatch), answers.back().data[0]);

    AppRecord record = {};
    recordRead(&record);
    EXPECT_NE(kRecordMagic, record.magic);
}

TEST_F(OtaReceiverTest, EndStagesTheImageAndWritesTheRecord)
{
    const auto image = makeImage();
    const uint32_t crc = crc32(image.data(), IMAGE_LENGTH);

    sendBegin(image, crc);
    sendAllData(image);

    receiver.handle(makeEnd(IMAGE_PACKETS - 1));

    ASSERT_EQ(1U + IMAGE_PACKETS + 1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Ack), answers.back().kind);
    EXPECT_EQ(IMAGE_PACKETS - 1, answers.back().seq);
    EXPECT_EQ(0, std::memcmp(image.data(), gStagingSector, IMAGE_LENGTH));

    AppRecord record = {};
    recordRead(&record);
    EXPECT_EQ(kRecordMagic, record.magic);
    EXPECT_EQ(kRecordFormat, record.format);
    EXPECT_EQ(kRecordFlagCopyPending, record.flags);
    EXPECT_EQ(IMAGE_LENGTH, record.length);
    EXPECT_EQ(crc, record.crc);
}

TEST_F(OtaReceiverTest, EndReportsARecordWriteFailure)
{
    const auto image = makeImage();
    sendBegin(image);
    sendAllData(image);
    flashFailureForTest() = true;

    receiver.handle(makeEnd(IMAGE_PACKETS - 1));

    ASSERT_EQ(1U + IMAGE_PACKETS + 1U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Error), answers.back().kind);
    EXPECT_EQ(static_cast<uint8_t>(OtaErrorCode::FlashFailure), answers.back().data[0]);
}

TEST_F(OtaReceiverTest, ARepeatedBeginIsAcknowledgedAgain)
{
    const auto image = makeImage();
    sendBegin(image);
    std::memset(gStagingSector, 0xAA, sizeof(gStagingSector));

    sendBegin(image);

    ASSERT_EQ(2U, answers.size());
    EXPECT_EQ(static_cast<uint8_t>(OtaKind::Ack), answers[1].kind);
    EXPECT_EQ(kOtaBeginAck, answers[1].seq);
}

TEST_F(OtaReceiverTest, AckAndErrorPacketsFromTheSenderAreIgnored)
{
    OtaPacket ack = {};
    ack.kind = static_cast<uint8_t>(OtaKind::Ack);

    OtaPacket error = {};
    error.kind = static_cast<uint8_t>(OtaKind::Error);

    receiver.handle(ack);
    receiver.handle(error);

    EXPECT_TRUE(answers.empty());
}

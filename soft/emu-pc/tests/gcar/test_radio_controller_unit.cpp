#include "CommandPacket.h"
#include "Events.h"
#include "RadioController.h"
#include "RadioProtocol.h"
#include "config_radio.h"
#include "mock_RF24.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::_;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::StrictMock;

namespace
{
constexpr uint8_t kQueueCapacity = 10;

bool same5(const uint8_t *lhs, const uint8_t *rhs)
{
    return std::memcmp(lhs, rhs, 5) == 0;
}

class RadioControllerUnitTest : public ::testing::Test {
protected:
    EventQueue appQueue;
    StrictMock<MockRF24> rf{ 10000000 };
    RadioController controller{ rf, &appQueue, gbox_pipe_name, gcar_pipe_name };

    static Event makeAction(CommandId cmd)
    {
        return Event{ EventId::ACTION, cmd };
    }

    void fillRadioQueue()
    {
        for (uint8_t i = 0; i < kQueueCapacity; ++i) {
            controller.sendEvent(makeAction(CommandId::MOTOR_FORWARD));
        }
        ASSERT_TRUE(controller.getRadioQueue()->IsFull());
    }

    void fillAppQueue()
    {
        for (uint8_t i = 0; i < kQueueCapacity; ++i) {
            Event evt = makeAction(CommandId::MOTOR_FORWARD);
            ASSERT_TRUE(appQueue.enqueue(evt));
        }
        ASSERT_TRUE(appQueue.IsFull());
    }
};
}

TEST_F(RadioControllerUnitTest, ProcessRadioTransmitsQueuedEvent)
{
    Event evt = makeAction(CommandId::MOTOR_TURN_LEFT);
    controller.sendEvent(evt);

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(false));
    EXPECT_CALL(rf, _stopListening());
    EXPECT_CALL(rf, _write(_, Event::packetSize()))
            .WillOnce(DoAll(Invoke([&evt](const void *buf, uint8_t len) {
                                ASSERT_EQ(len, Event::packetSize());
                                EventPacket expected{};
                                evt.toPacket(expected);
                                expected.tag = static_cast<uint8_t>(PacketTag::Event);
                                EXPECT_EQ(std::memcmp(buf, &expected, len), 0);
                            }),
                            Return(true)));
    EXPECT_CALL(rf, _startListening());

    controller.processRadio();
    EXPECT_TRUE(controller.getRadioQueue()->IsEmpty());
}

TEST_F(RadioControllerUnitTest, ProcessRadioDecodesCommandPacketIntoActionEvents)
{
    CommandPacket packet{};
    packet.count = 3;
    packet.commands[0] = static_cast<uint8_t>(CommandId::MOTOR_FORWARD);
    packet.commands[1] = static_cast<uint8_t>(CommandId::MOTOR_TURN_RIGHT);
    packet.commands[2] = static_cast<uint8_t>(CommandId::MOTOR_STOP);

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
            .WillOnce(Invoke([&packet](void *buf, uint8_t len) {
                ASSERT_EQ(len, sizeof(CommandPacket));
                std::memcpy(buf, &packet, sizeof(packet));
            }));

    controller.processRadio();

    Event e0{}, e1{}, e2{};
    ASSERT_TRUE(appQueue.dequeue(e0));
    ASSERT_TRUE(appQueue.dequeue(e1));
    ASSERT_TRUE(appQueue.dequeue(e2));

    EXPECT_EQ(e0.getEventId(), EventId::ACTION);
    EXPECT_EQ(e0.getPayload(), CommandId::MOTOR_FORWARD);
    EXPECT_EQ(e1.getEventId(), EventId::ACTION);
    EXPECT_EQ(e1.getPayload(), CommandId::MOTOR_TURN_RIGHT);
    EXPECT_EQ(e2.getEventId(), EventId::ACTION);
    EXPECT_EQ(e2.getPayload(), CommandId::MOTOR_STOP);
}

TEST_F(RadioControllerUnitTest, ProcessRadioDecodesTaggedEventPacket)
{
    EventPacket packet{};
    packet.tag = static_cast<uint8_t>(PacketTag::Event);
    packet.eventId = static_cast<uint8_t>(EventId::ACTION);
    packet.payload = static_cast<uint8_t>(CommandId::MOTOR_STOP);

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
            .WillOnce(Invoke([&packet](void *buf, uint8_t len) {
                ASSERT_EQ(len, sizeof(CommandPacket));
                std::memset(buf, 0, len);
                std::memcpy(buf, &packet, sizeof(packet));
            }));

    controller.processRadio();

    EXPECT_EQ(appQueue.GetSize(), 1);
}

TEST_F(RadioControllerUnitTest, ProcessRadioRoutesTaggedTelemetryWithoutQueuing)
{
    TelemetryPacket telemetry{};
    telemetry.tag = static_cast<uint8_t>(PacketTag::Telemetry);
    telemetry.yaw = 12.5f;

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
            .WillOnce(Invoke([&telemetry](void *buf, uint8_t len) {
                std::memset(buf, 0, len);
                std::memcpy(buf, &telemetry, sizeof(telemetry));
            }));

    testing::internal::CaptureStdout();
    controller.processRadio();
    const std::string out = testing::internal::GetCapturedStdout();

    /* Telemetry is watched, never turned into an action event */
    EXPECT_TRUE(appQueue.IsEmpty());
    EXPECT_NE(out.find("Telemetry"), std::string::npos);
}

TEST_F(RadioControllerUnitTest, ProcessRadioReassemblesLogFragments)
{
    uint8_t first[32] = {};
    uint8_t last[32] = {};

    first[0] = static_cast<uint8_t>(PacketTag::Log);
    first[1] = static_cast<uint8_t>(0x80 | 6); /* continuation, 6 bytes */
    std::memcpy(&first[2], "hello ", 6);

    last[0] = static_cast<uint8_t>(PacketTag::Log);
    last[1] = 5; /* final piece, 5 bytes */
    std::memcpy(&last[2], "world", 5);

    {
        InSequence seq;
        EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
        EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
                .WillOnce(
                        Invoke([&first](void *buf, uint8_t len) { std::memcpy(buf, first, len); }));
        EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
        EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
                .WillOnce(Invoke([&last](void *buf, uint8_t len) { std::memcpy(buf, last, len); }));
    }

    /* Nothing is printed while the line is only half received */
    testing::internal::CaptureStdout();
    controller.processRadio();
    const std::string halfway = testing::internal::GetCapturedStdout();
    EXPECT_EQ(halfway.find("hello"), std::string::npos);

    testing::internal::CaptureStdout();
    controller.processRadio();
    const std::string done = testing::internal::GetCapturedStdout();
    EXPECT_NE(done.find("hello world"), std::string::npos);
    EXPECT_TRUE(appQueue.IsEmpty());
}

TEST_F(RadioControllerUnitTest, ProcessRadioDropsUnknownPacketTag)
{
    uint8_t unknown[32] = {};
    unknown[0] = 0xF0; /* neither a command count nor a known tag */
    unknown[1] = static_cast<uint8_t>(CommandId::MOTOR_FORWARD);

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
            .WillOnce(
                    Invoke([&unknown](void *buf, uint8_t len) { std::memcpy(buf, unknown, len); }));

    testing::internal::CaptureStdout();
    controller.processRadio();
    const std::string out = testing::internal::GetCapturedStdout();

    EXPECT_TRUE(appQueue.IsEmpty());
    EXPECT_NE(out.find("unknown packet tag"), std::string::npos);
}

TEST_F(RadioControllerUnitTest, CommandPacketRxStopsWhenAppQueueIsFull)
{
    fillAppQueue();

    CommandPacket packet{};
    packet.count = 2;
    packet.commands[0] = static_cast<uint8_t>(CommandId::MOTOR_FORWARD);
    packet.commands[1] = static_cast<uint8_t>(CommandId::MOTOR_BACKWARD);

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
            .WillOnce(Invoke([&packet](void *buf, uint8_t len) {
                ASSERT_EQ(len, sizeof(CommandPacket));
                std::memcpy(buf, &packet, sizeof(packet));
            }));

    controller.processRadio();

    EXPECT_TRUE(appQueue.IsFull());
}

TEST_F(RadioControllerUnitTest, SendEventDropsWhenRadioQueueIsFull)
{
    fillRadioQueue();

    controller.sendEvent(makeAction(CommandId::MOTOR_STOP));

    EXPECT_EQ(controller.getRadioQueue()->GetSize(), kQueueCapacity);
}

TEST_F(RadioControllerUnitTest, SendBufferReturnsFalseOnWriteFailure)
{
    uint8_t payload[4] = { 1, 2, 3, 4 };

    EXPECT_CALL(rf, _stopListening());
    EXPECT_CALL(rf, _write(payload, sizeof(payload))).WillOnce(Return(false));
    EXPECT_CALL(rf, _startListening());

    EXPECT_FALSE(controller.sendBuffer(payload, sizeof(payload)));
}

TEST_F(RadioControllerUnitTest, SendTelemetryStaysOnCommandPipeAndStampsTag)
{
    TelemetryPacket telemetry{};
    bool stamped = false;

    /* No address switch any more: telemetry rides the writing pipe like
     * every other packet this device sends */
    EXPECT_CALL(rf, _openWritingPipe(_)).Times(0);

    {
        InSequence seq;
        EXPECT_CALL(rf, _stopListening());
        EXPECT_CALL(rf, _write(_, sizeof(TelemetryPacket)))
                .WillOnce(Invoke([&stamped](const void *buf, uint8_t len) {
                    TelemetryPacket sent{};
                    /* A wrong length leaves stamped false, which fails the
                     * assertion below just as loudly */
                    if (len == sizeof(sent)) {
                        std::memcpy(&sent, buf, sizeof(sent));
                        stamped = (sent.tag == static_cast<uint8_t>(PacketTag::Telemetry));
                    }
                    return true;
                }));
        EXPECT_CALL(rf, _startListening());
    }

    EXPECT_TRUE(controller.sendTelemetry(telemetry));
    EXPECT_TRUE(stamped);
    /* The tag is stamped on the copy: the caller's packet is untouched */
    EXPECT_EQ(telemetry.tag, 0);
}

TEST_F(RadioControllerUnitTest, SendTelemetryFailurePropagatesWithoutPipeSwitch)
{
    TelemetryPacket telemetry{};

    EXPECT_CALL(rf, _openWritingPipe(_)).Times(0);

    {
        InSequence seq;
        EXPECT_CALL(rf, _stopListening());
        EXPECT_CALL(rf, _write(_, sizeof(TelemetryPacket))).WillOnce(Return(false));
        EXPECT_CALL(rf, _startListening());
    }

    EXPECT_FALSE(controller.sendTelemetry(telemetry));
}

TEST_F(RadioControllerUnitTest, InitReturnsEarlyWhenRadioBeginFails)
{
    EXPECT_CALL(rf, _begin()).WillOnce(Return(false));

    EXPECT_CALL(rf, _setPALevel(_, _)).Times(0);
    EXPECT_CALL(rf, _setDataRate(_)).Times(0);
    EXPECT_CALL(rf, _setAddressWidth(_)).Times(0);
    EXPECT_CALL(rf, _setPayloadSize(_)).Times(0);
    EXPECT_CALL(rf, _openWritingPipe(_)).Times(0);
    EXPECT_CALL(rf, _openReadingPipe(_, _)).Times(0);
    EXPECT_CALL(rf, _powerUp()).Times(0);
    EXPECT_CALL(rf, _startListening()).Times(0);

    controller.init();
}

TEST_F(RadioControllerUnitTest, InvalidCommandPacketCountIsIgnored)
{
    CommandPacket invalidZero{};
    invalidZero.count = 0;

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
            .WillOnce(Invoke([&invalidZero](void *buf, uint8_t len) {
                ASSERT_EQ(len, sizeof(CommandPacket));
                std::memcpy(buf, &invalidZero, sizeof(invalidZero));
            }));

    controller.processRadio();
    EXPECT_TRUE(appQueue.IsEmpty());

    CommandPacket invalidOverflow{};
    invalidOverflow.count = static_cast<uint8_t>(BOX_MAX_PAYLOAD_COMMANDS + 1);
    invalidOverflow.commands[0] = static_cast<uint8_t>(CommandId::MOTOR_FORWARD);

    EXPECT_CALL(rf, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rf, _read(_, sizeof(CommandPacket)))
            .WillOnce(Invoke([&invalidOverflow](void *buf, uint8_t len) {
                ASSERT_EQ(len, sizeof(CommandPacket));
                std::memcpy(buf, &invalidOverflow, sizeof(invalidOverflow));
            }));

    controller.processRadio();
    EXPECT_TRUE(appQueue.IsEmpty());
}

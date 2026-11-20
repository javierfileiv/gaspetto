#include "Arduino.h"
#include "Context.h"
#include "GBox.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "RadioController.h"
#include "WireVersion.h"
#include "board_presets.h"
#include "config_radio.h"
#include "mock_RF24.h"

#include <cstdlib>
#include <cstring>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::InSequence;
using testing::Return;
using testing::StrictMock;

namespace
{
class TestableGBox : public GBox {
public:
    explicit TestableGBox(Context &ctx)
            : GBox(ctx)
    {
    }

    void serviceAnimation()
    {
        serviceProgramAnimation();
    }

    void servicePanel()
    {
        serviceStatus();
    }

    void showHealth(bool ok)
    {
        showHealthStatus(ok);
    }

    void setSystemOk(bool ok)
    {
        systemOk_ = ok;
    }

    const char *pieceName(BoxPieceId piece) const
    {
        return pieceToString(piece);
    }

protected:
    void delayMs(int) const override
    {
    }
};

Event makeButtonPressedEvent()
{
    return Event{ EventId::BUTTON_PRESSED, CommandId::NONE };
}

class ScopedEnvFlag {
public:
    explicit ScopedEnvFlag(const char *name)
            : name_(name)
    {
        setenv(name_, "1", 1);
    }

    ~ScopedEnvFlag()
    {
        unsetenv(name_);
    }

private:
    const char *name_;
};

/* The base delayMs() has to stay covered, and running an animation for real
 * would now cost five seconds: expose the method and call it once */
class DelayExposingGBox : public GBox {
public:
    explicit DelayExposingGBox(Context &ctx)
            : GBox(ctx)
    {
    }

    void callBaseDelay() const
    {
        delayMs(1);
    }
};

struct BoxTestRig {
    EventQueue eventQueue;
    TimeredEventQueue timeredEventQueue;
    IdleState idleState;
    ProcessingState processingState;
    StrictMock<MockRF24> mockRf24{ 10000000 };
    RadioController radioController;
    Context ctx;
    TestableGBox box;

    BoxTestRig()
            : radioController(mockRf24, &eventQueue, gcar_pipe_name, gbox_pipe_name)
            , ctx{ &eventQueue, &timeredEventQueue, &radioController, &idleState, &processingState }
            , box(ctx)
    {
        /* The ADS emulation moves the clock forward by 100 ms per conversion,
         * and the box pings the car once a second. Start every test from a
         * known time, or a test that forbids radio traffic would see a ping
         * left over from the scan of a previous one */
        microsCounter.store(0);

        /* work() reads the radio on every pass, so a silent receiver is the
         * default here. A test that wants a packet declares its own
         * expectation, and gmock gives it precedence over this one */
        EXPECT_CALL(mockRf24, _available(_)).WillRepeatedly(Return(false));
        box.setLowPowerModeCallback([]() {});
        box.init(StateId::IDLE);
    }
};
}

TEST(GBoxFsmTest, ButtonPressRunsProcessingAndReturnsToIdle)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gbox_test::injectDemoBoard(rig.box);

    {
        InSequence seq;
        EXPECT_CALL(rig.mockRf24, _stopListening());
        EXPECT_CALL(rig.mockRf24, _startWrite(_, sizeof(CommandPacket), false))
                .WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _startListening());
    }

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, EmptyBoardStaysIdleWithoutRadioSend)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gbox_test::injectEmptyBoard(rig.box);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, RadioFailureStillReturnsToIdle)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gbox_test::injectDemoBoard(rig.box);

    {
        InSequence seq;
        EXPECT_CALL(rig.mockRf24, _stopListening());
        EXPECT_CALL(rig.mockRf24, _startWrite(_, sizeof(CommandPacket), false))
                .WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).WillOnce(Return(false));
        EXPECT_CALL(rig.mockRf24, _startListening());
    }

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, OverflowBoardBuildFailsWithoutRadioSend)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gbox_test::injectOverflowBoard(rig.box);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, InitHardwareNotTwice)
{
    BoxTestRig rig;

    EXPECT_CALL(rig.mockRf24, _begin()).WillOnce(Return(true));
    EXPECT_CALL(rig.mockRf24, _setPALevel(_, _));
    EXPECT_CALL(rig.mockRf24, _setDataRate(_)).WillOnce(Return(true));
    EXPECT_CALL(rig.mockRf24, _setAddressWidth(_));
    EXPECT_CALL(rig.mockRf24, _setRetries(2, 3));
    EXPECT_CALL(rig.mockRf24, _setPayloadSize(_));
    EXPECT_CALL(rig.mockRf24, _openWritingPipe(_));
    EXPECT_CALL(rig.mockRf24, _openReadingPipe(_, _));
    EXPECT_CALL(rig.mockRf24, _powerUp());
    EXPECT_CALL(rig.mockRf24, _startListening());

    rig.box.initHardware();
    rig.box.initHardware();
}

TEST(GBoxFsmTest, SendProgramReturnsFalseWhenRadioControllerMissing)
{
    EventQueue eventQueue;
    TimeredEventQueue timeredEventQueue;
    IdleState idleState;
    ProcessingState processingState;
    Context ctx{ &eventQueue, &timeredEventQueue, nullptr, &idleState, &processingState };
    TestableGBox box(ctx);

    CommandPacket packet{};
    packet.count = 1;
    packet.commands[0] = static_cast<uint8_t>(CommandId::MOTOR_FORWARD);

    EXPECT_FALSE(box.sendProgram(packet));
}

TEST(GBoxFsmTest, PostEventFailsWhenQueueMissing)
{
    TimeredEventQueue timeredEventQueue;
    IdleState idleState;
    ProcessingState processingState;
    Context ctx{ nullptr, &timeredEventQueue, nullptr, &idleState, &processingState };
    TestableGBox box(ctx);

    EXPECT_EQ(box.postEvent(makeButtonPressedEvent()), -1);
}

TEST(GBoxFsmTest, DebouncePathHandlesFullQueueInEmulation)
{
    BoxTestRig rig;

    while (!rig.eventQueue.IsFull()) {
        Event fillEvt{ EventId::ACTION, CommandId::MOTOR_FORWARD };
        ASSERT_TRUE(rig.eventQueue.enqueue(fillEvt));
    }

    Event evt{ EventId::BUTTON_PRESSED, CommandId::NONE };
    rig.box.debounceAndEnqueue(evt, 1234UL);

    EXPECT_TRUE(rig.eventQueue.IsFull());
}

TEST(GBoxFsmTest, DebouncePathEnqueuesWhenQueueHasRoom)
{
    BoxTestRig rig;

    ASSERT_TRUE(rig.eventQueue.IsEmpty());

    Event evt{ EventId::BUTTON_PRESSED, CommandId::NONE };
    rig.box.debounceAndEnqueue(evt, 1234UL);

    ASSERT_FALSE(rig.eventQueue.IsEmpty());
    Event dequeued{};
    ASSERT_TRUE(rig.eventQueue.dequeue(dequeued));
    EXPECT_EQ(dequeued.getEventId(), EventId::BUTTON_PRESSED);
}

TEST(GBoxFsmTest, InjectRawAdcValuesCanProduceInvalidPieces)
{
    BoxTestRig rig;

    std::array<uint16_t, BOX_TOTAL_SLOTS> rawValues{};
    rawValues.fill(0);
    rawValues[0] = 5000; /* Out-of-range => INVALID piece path. */

    rig.box.injectRawAdcValues(rawValues);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, InjectBoardPiecesHandlesUnknownPieceValue)
{
    BoxTestRig rig;

    BoxPieceId board[BOX_TOTAL_SLOTS];
    gbox_test::fillEmptyBoard(board);
    board[0] = static_cast<BoxPieceId>(0xFF);

    rig.box.injectBoardPieces(board);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, BaseDelayPathIsExercisedByConcreteBox)
{
    EventQueue eventQueue;
    TimeredEventQueue timeredEventQueue;
    IdleState idleState;
    ProcessingState processingState;
    Context ctx{ &eventQueue, &timeredEventQueue, nullptr, &idleState, &processingState };
    DelayExposingGBox box(ctx);

    /* Uses the base delayMs() implementation (no test override). */
    box.callBaseDelay();
}

TEST(GBoxFsmTest, IdleStateIgnoresNonButtonEvents)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    Event evt{ EventId::ACTION, CommandId::MOTOR_FORWARD };
    ASSERT_EQ(rig.box.postEvent(evt), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, AdsBeginFailurePathProducesBuildErrorWithoutRadioSend)
{
    BoxTestRig rig;

    setenv("ADS_BEGIN_FAIL", "1", 1);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);

    unsetenv("ADS_BEGIN_FAIL");
}

TEST(GBoxFsmTest, NegativeAdsSamplesTriggerRecoveryAndBuildErrorWithoutRadioSend)
{
    BoxTestRig rig;
    ScopedEnvFlag forceNegativeRaw("ADS_FORCE_NEGATIVE_RAW");

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, StuckAdsConversionTriggersTimeoutRecoveryAndBuildErrorWithoutRadioSend)
{
    BoxTestRig rig;
    ScopedEnvFlag stuckConversion("ADS_CONVERSION_STUCK");

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GBoxFsmTest, WorkReadsARadioPacketWithoutAnsweringTelemetry)
{
    BoxTestRig rig;
    TelemetryPacket telemetry = {};

    telemetry.tag = static_cast<uint8_t>(PacketTag::Telemetry);
    rig.mockRf24.simulateReceivedPacket(1, &telemetry, sizeof(telemetry));

    EXPECT_CALL(rig.mockRf24, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rig.mockRf24, _read(_, sizeof(CommandPacket)));
    /* Telemetry is watched, not acted on, so receiving it must not send
     * anything back over the channel the car is already using */
    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _startWrite(_, _, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

namespace
{
void expectProgramSend(BoxTestRig &rig)
{
    EXPECT_CALL(rig.mockRf24, _stopListening());
    EXPECT_CALL(rig.mockRf24, _startWrite(_, sizeof(CommandPacket), false)).WillOnce(Return(true));
    EXPECT_CALL(rig.mockRf24, _txStandBy(_, _)).WillOnce(Return(true));
    EXPECT_CALL(rig.mockRf24, _startListening());
}

CommandPacket forwardProgram()
{
    CommandPacket packet{};

    packet.count = 1;
    packet.commands[0] = static_cast<uint8_t>(CommandId::MOTOR_FORWARD);
    return packet;
}
} /* namespace */

TEST(GBoxFsmTest, HealthCheckMarksTheSystemOk)
{
    BoxTestRig rig;

    EXPECT_TRUE(rig.box.runHealthCheck());
}

TEST(GBoxFsmTest, HealthStatusLightsTheBuildPixel)
{
    BoxTestRig rig;

    rig.box.showHealth(true);
    rig.box.showHealth(false);
}

TEST(GBoxFsmTest, ProgramDonePlaysTheGreenPulse)
{
    BoxTestRig rig;

    rig.box.runProgramDoneAnimation();
}

TEST(GBoxFsmTest, AnAppendedProgramKeepsTheBlink)
{
    BoxTestRig rig;
    const CommandPacket packet = forwardProgram();

    expectProgramSend(rig);
    ASSERT_TRUE(rig.box.sendProgram(packet));

    expectProgramSend(rig);
    ASSERT_TRUE(rig.box.sendProgram(packet));

    EXPECT_TRUE(rig.box.isProgramRunning());
}

TEST(GBoxFsmTest, StatusPanelKeepsTheBoxAndLinkPixels)
{
    BoxTestRig rig;
    const CommandPacket packet = forwardProgram();

    expectProgramSend(rig);
    ASSERT_TRUE(rig.box.sendProgram(packet));

    microsCounter.fetch_add(200U * 1000U);
    rig.box.servicePanel();
    EXPECT_TRUE(rig.box.isProgramRunning());

    rig.box.runProgramDoneAnimation();
    EXPECT_FALSE(rig.box.isProgramRunning());

    microsCounter.fetch_add(200U * 1000U);
    rig.box.servicePanel();

    microsCounter.store(0);
}

TEST(GBoxFsmTest, VersionAnswerTurnsTheMiddlePixelGreen)
{
    BoxTestRig rig;
    VersionPacket answer = {};

    answer.tag = static_cast<uint8_t>(PacketTag::Version);
    answer.wire = kWireProtocolVersion;
    answer.build = 42;
    std::strncpy(answer.version, "abc1234", sizeof(answer.version) - 1);

    microsCounter.store(1000U * 1000U);
    rig.mockRf24.simulateReceivedPacket(1, &answer, sizeof(answer));

    EXPECT_CALL(rig.mockRf24, _available(_)).WillOnce(Return(true));
    EXPECT_CALL(rig.mockRf24, _read(_, sizeof(CommandPacket)));
    expectProgramSend(rig);

    rig.box.work();

    microsCounter.store(0);
}

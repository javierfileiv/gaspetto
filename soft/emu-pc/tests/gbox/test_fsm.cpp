#include "Context.h"
#include "GBox.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "RadioController.h"
#include "board_presets.h"
#include "config_radio.h"
#include "mock_RF24.h"

#include <cstdlib>
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
            : radioController(mockRf24, &eventQueue, gbox_pipe_name, gcar_pipe_name)
            , ctx{ &eventQueue, &timeredEventQueue, &radioController, &idleState, &processingState }
            , box(ctx)
    {
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
        EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(true));
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
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
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
        EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(false));
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
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
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
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
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
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
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
    GBox box(ctx);

    /* Uses base delayMs() implementation (no test override). */
    box.runScanAnimation();
}

TEST(GBoxFsmTest, IdleStateIgnoresNonButtonEvents)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
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
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
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
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
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
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

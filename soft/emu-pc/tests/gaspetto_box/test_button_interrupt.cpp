#include "Context.h"
#include "GaspettoBox.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "RadioController.h"
#include "board_presets.h"
#include "config_radio.h"
#include "mock_RF24.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::InSequence;
using testing::Return;
using testing::StrictMock;

namespace
{
class TestableGaspettoBox : public GaspettoBox {
public:
    explicit TestableGaspettoBox(Context &ctx)
            : GaspettoBox(ctx)
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

struct BoxTestRig {
    EventQueue eventQueue;
    TimeredEventQueue timeredEventQueue;
    IdleState idleState;
    ProcessingState processingState;
    StrictMock<MockRF24> mockRf24{ 10000000 };
    RadioController radioController;
    Context ctx;
    TestableGaspettoBox box;

    BoxTestRig()
            : radioController(mockRf24, &eventQueue, gaspetto_box_pipe_name, gcar_pipe_name)
            , ctx{ &eventQueue, &timeredEventQueue, &radioController, &idleState, &processingState }
            , box(ctx)
    {
        box.setLowPowerModeCallback([]() {});
        box.init(StateId::IDLE);
    }
};
}

/* ============================================================================ */
/* Test: isCurrentlyScanning() returns correct state                           */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, IsCurrentlyScanningShouldReturnFalseByDefault)
{
    BoxTestRig rig;

    EXPECT_FALSE(rig.box.isCurrentlyScanning());
}

/* ============================================================================ */
/* Test: Button interrupt during processing sends clear queue command         */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, ButtonPressedDuringProcessingSendsClearQueueCommand)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gaspetto_box_test::injectDemoBoard(rig.box);

    /* First button press starts processing */
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

/* ============================================================================ */
/* Test: sendClearQueueCommand() creates correct packet with QUEUE_CLEAR      */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, SendClearQueueCommandCreatesCorrectPacket)
{
    BoxTestRig rig;

    gaspetto_box_test::injectDemoBoard(rig.box);

    InSequence seq;
    EXPECT_CALL(rig.mockRf24, _stopListening());
    EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket)))
            .WillOnce([](const void *data, uint8_t len) {
                EXPECT_EQ(len, sizeof(CommandPacket));
                const CommandPacket *pkt = static_cast<const CommandPacket *>(data);
                EXPECT_EQ(pkt->count, 2);
                EXPECT_EQ(pkt->commands[0], static_cast<uint8_t>(CommandId::QUEUE_CLEAR));
                EXPECT_EQ(pkt->commands[1], static_cast<uint8_t>(CommandId::MOTOR_STOP));
                return true;
            });
    EXPECT_CALL(rig.mockRf24, _startListening());

    bool result = rig.box.sendClearQueueCommand();

    EXPECT_TRUE(result);
}

/* ============================================================================ */
/* Test: sendClearQueueCommand() returns false when radio controller is null  */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, SendClearQueueCommandReturnsFalseWhenRadioNull)
{
    EventQueue eventQueue;
    TimeredEventQueue timeredEventQueue;
    IdleState idleState;
    ProcessingState processingState;
    Context ctx{ &eventQueue, &timeredEventQueue, nullptr, &idleState, &processingState };

    TestableGaspettoBox box(ctx);
    box.setLowPowerModeCallback([]() {});
    box.init(StateId::IDLE);

    bool result = box.sendClearQueueCommand();

    EXPECT_FALSE(result);
}

/* ============================================================================ */
/* Test: interruptScanning() sets scanning flag to false                       */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, InterruptScanningSetsFlagToFalse)
{
    BoxTestRig rig;

    /* Initially should not be scanning */
    EXPECT_FALSE(rig.box.isCurrentlyScanning());

    /* Start scanning */
    rig.box.startScanning();

    /* Should now be scanning */
    EXPECT_TRUE(rig.box.isCurrentlyScanning());

    /* Interrupt scanning */
    rig.box.interruptScanning();

    /* Should no longer be scanning */
    EXPECT_FALSE(rig.box.isCurrentlyScanning());
}

TEST(GaspettoBoxButtonInterruptTest, ProcessingStateButtonPressInterruptsScanAndClearsQueue)
{
    BoxTestRig rig;
    rig.box.startScanning();

    ASSERT_TRUE(rig.box.isCurrentlyScanning());

    InSequence seq;
    EXPECT_CALL(rig.mockRf24, _stopListening());
    EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(true));
    EXPECT_CALL(rig.mockRf24, _startListening());

    Event evt = makeButtonPressedEvent();
    rig.processingState.processEvent(evt);

    EXPECT_FALSE(rig.box.isCurrentlyScanning());
    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

TEST(GaspettoBoxButtonInterruptTest, ProcessingStateIgnoresNonButtonInterruptEvent)
{
    BoxTestRig rig;

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    Event evt{ EventId::ACTION, CommandId::MOTOR_FORWARD };
    rig.processingState.processEvent(evt);

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

/* ============================================================================ */
/* Test: Button press in idle state transitions to processing (wake-up)       */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, ButtonPressInIdleTransitionsToProcessing)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gaspetto_box_test::injectDemoBoard(rig.box);

    {
        InSequence seq;
        EXPECT_CALL(rig.mockRf24, _stopListening());
        EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _startListening());
    }

    rig.box.postEvent(makeButtonPressedEvent());
    rig.box.work();

    /* Should return to idle after processing */
    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

/* ============================================================================ */
/* Test: Multiple button presses                                              */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, MultipleButtonPressesWorkCorrectly)
{
    BoxTestRig rig;

    gaspetto_box_test::injectDemoBoard(rig.box);

    /* First button press */
    {
        InSequence seq;
        EXPECT_CALL(rig.mockRf24, _stopListening());
        EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _startListening());
    }

    rig.box.postEvent(makeButtonPressedEvent());
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);

    /* Second button press - re-inject board since it was processed */
    gaspetto_box_test::injectDemoBoard(rig.box);

    {
        InSequence seq;
        EXPECT_CALL(rig.mockRf24, _stopListening());
        EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _startListening());
    }

    rig.box.postEvent(makeButtonPressedEvent());
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

/* ============================================================================ */
/* Test: Empty board with button press                                        */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, EmptyBoardWithButtonPressDoesNotSend)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gaspetto_box_test::injectEmptyBoard(rig.box);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

/* ============================================================================ */
/* Test: Radio failure during normal operation                                */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, RadioFailureDuringNormalOperation)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gaspetto_box_test::injectDemoBoard(rig.box);

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

/* ============================================================================ */
/* Test: Clear queue command sends QUEUE_CLEAR first, then MOTOR_STOP        */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, ClearQueueCommandOrderIsCorrect)
{
    BoxTestRig rig;

    InSequence seq;
    EXPECT_CALL(rig.mockRf24, _stopListening());
    EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket)))
            .WillOnce([](const void *data, uint8_t len) {
                const CommandPacket *pkt = static_cast<const CommandPacket *>(data);
                EXPECT_EQ(pkt->commands[0], static_cast<uint8_t>(CommandId::QUEUE_CLEAR));
                EXPECT_EQ(pkt->commands[1], static_cast<uint8_t>(CommandId::MOTOR_STOP));
                return true;
            });
    EXPECT_CALL(rig.mockRf24, _startListening());

    bool result = rig.box.sendClearQueueCommand();

    EXPECT_TRUE(result);
}

/* ============================================================================ */
/* Test: Interrupted scan returns to idle                                     */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, InterruptedScanTransitionsToIdle)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gaspetto_box_test::injectDemoBoard(rig.box);

    {
        InSequence seq;
        EXPECT_CALL(rig.mockRf24, _stopListening());
        EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _startListening());
    }

    rig.box.postEvent(makeButtonPressedEvent());
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

/* ============================================================================ */
/* Test: Overflow board with button press                                     */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, OverflowBoardWithButtonPressFails)
{
    BoxTestRig rig;

    ASSERT_EQ(rig.box.getCurrentState(), &rig.idleState);

    gaspetto_box_test::injectOverflowBoard(rig.box);

    EXPECT_CALL(rig.mockRf24, _stopListening()).Times(0);
    EXPECT_CALL(rig.mockRf24, _write(_, _)).Times(0);
    EXPECT_CALL(rig.mockRf24, _startListening()).Times(0);

    ASSERT_EQ(rig.box.postEvent(makeButtonPressedEvent()), 0);
    rig.box.work();

    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

/* ============================================================================ */
/* Test: Clear queue command includes both QUEUE_CLEAR and MOTOR_STOP         */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, ClearQueueCommandIncludesBothCommands)
{
    BoxTestRig rig;

    bool capturedResult = false;
    uint8_t capturedCmd0 = 0;
    uint8_t capturedCmd1 = 0;

    InSequence seq;
    EXPECT_CALL(rig.mockRf24, _stopListening());
    EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket)))
            .WillOnce([&](const void *data, uint8_t len) {
                const CommandPacket *pkt = static_cast<const CommandPacket *>(data);
                capturedCmd0 = pkt->commands[0];
                capturedCmd1 = pkt->commands[1];
                return true;
            });
    EXPECT_CALL(rig.mockRf24, _startListening());

    capturedResult = rig.box.sendClearQueueCommand();

    EXPECT_TRUE(capturedResult);
    EXPECT_EQ(capturedCmd0, static_cast<uint8_t>(CommandId::QUEUE_CLEAR));
    EXPECT_EQ(capturedCmd1, static_cast<uint8_t>(CommandId::MOTOR_STOP));
}

/* ============================================================================ */
/* Test: Processing state handles button press event                          */
/* ============================================================================ */
TEST(GaspettoBoxButtonInterruptTest, ProcessingStateHandlesButtonPressEvent)
{
    BoxTestRig rig;

    gaspetto_box_test::injectDemoBoard(rig.box);

    /* Transition to processing */
    {
        InSequence seq;
        EXPECT_CALL(rig.mockRf24, _stopListening());
        EXPECT_CALL(rig.mockRf24, _write(_, sizeof(CommandPacket))).WillOnce(Return(true));
        EXPECT_CALL(rig.mockRf24, _startListening());
    }

    rig.box.postEvent(makeButtonPressedEvent());
    rig.box.work();

    /* We should be back in idle state */
    EXPECT_EQ(rig.box.getCurrentState(), &rig.idleState);
}

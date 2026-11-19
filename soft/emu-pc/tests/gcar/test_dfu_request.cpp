/*
 * DFU request tests: the radio carries a tagged event that the car turns into
 * a motor stop plus a jump to the bootloader. The jump itself is wired by
 * main.cpp, so the tests substitute a counter through setDfuCallback and can
 * observe the sequence without an STM32.
 */
#include "CommandPacket.h"
#include "PacketTag.h"
#include "fixture.h"

#include <gtest/gtest.h>

namespace
{
int gDfuCalls = 0;

void countDfuCall()
{
    ++gDfuCalls;
}

/* A whole payload, zero padded: the receiver reads a full CommandPacket width
 * whatever the sender wrote, so the test feeds the same shape */
void fillDfuPacket(uint8_t (&packet)[sizeof(CommandPacket)])
{
    packet[0] = static_cast<uint8_t>(PacketTag::Event);
    packet[1] = static_cast<uint8_t>(EventId::DFU_REQUEST);
    packet[2] = static_cast<uint8_t>(CommandId::NONE);
}
} /* namespace */

class DfuRequestTest : public Fixture {
protected:
    void SetUp() override
    {
        Fixture::SetUp();
        gDfuCalls = 0;
        gaspettoCar.setDfuCallback(countDfuCall);
    }
};

TEST_F(DfuRequestTest, StopsTheMotorsAndCallsTheCallback)
{
    uint8_t packet[sizeof(CommandPacket)] = {};

    fillDfuPacket(packet);
    radio_receive_raw(packet, sizeof(packet));

    set_exit_low_power_mode();
    expect_update_movement();
    expect_stop_both_motors();
    gaspettoCar.work();

    EXPECT_EQ(gDfuCalls, 1);
    EXPECT_EQ(gaspettoCar.getCurrentState(), &idleState);
}

TEST_F(DfuRequestTest, WithoutACallbackTheRequestIsInert)
{
    uint8_t packet[sizeof(CommandPacket)] = {};

    gaspettoCar.setDfuCallback(nullptr);
    fillDfuPacket(packet);
    radio_receive_raw(packet, sizeof(packet));

    set_exit_low_power_mode();
    expect_update_movement();
    expect_stop_both_motors();
    gaspettoCar.work();

    EXPECT_EQ(gDfuCalls, 0);
    EXPECT_EQ(gaspettoCar.getCurrentState(), &idleState);
}

TEST_F(DfuRequestTest, InterruptsAMovement)
{
    uint8_t packet[sizeof(CommandPacket)] = {};

    radio_receive_event(&forwardEvent);
    set_exit_low_power_mode();
    expect_update_movement();
    expect_move_forward(INITIAL_MOTOR_SPEED, MOTOR_TIMEOUT_MS);
    expect_target_not_reached();
    gaspettoCar.work();
    ASSERT_EQ(gaspettoCar.getCurrentState(), &processingState);

    fillDfuPacket(packet);
    radio_receive_raw(packet, sizeof(packet));

    expect_update_movement();
    expect_stop_both_motors();
    gaspettoCar.work();

    EXPECT_EQ(gDfuCalls, 1);
}

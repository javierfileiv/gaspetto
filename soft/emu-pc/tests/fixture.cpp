#include "fixture.h"

#include "CommandId.h"

using testing::NotNull;
using testing::Return;
using testing::Invoke;
using testing::DoAll;

bool low_power_mode = false;

void enter_low_power_mode(void)
{
    SwitchToLowPowerMode();
}

void Fixture::expect_car_init()
{
    expect_movement_controller_init();
    expect_radio_initialization();
    EXPECT_CALL(_mock_arduino, SwitchToLowPowerMode);
}

void Fixture::expect_movement_controller_init()
{
    EXPECT_CALL(_mock_movementController, init(MOTOR_FREQ));
}

void Fixture::expect_enter_low_power_mode()
{
    EXPECT_CALL(_mock_arduino, SwitchToLowPowerMode);
}

void Fixture::set_exit_low_power_mode()
{
    low_power_mode = false;
}

void Fixture::expect_move_forward(float speed, uint32_t timeout_ms)
{
    EXPECT_CALL(_mock_movementController, startStraightDriving(speed, timeout_ms));
}

void Fixture::expect_move_backward(float speed, uint32_t timeout_ms)
{
    EXPECT_CALL(_mock_movementController, startStraightDriving(-speed, timeout_ms));
}

void Fixture::expect_turn_left(float target_yaw, float speed, uint32_t timeout_ms)
{
    EXPECT_CALL(_mock_movementController, startTurningInPlace(target_yaw, speed, timeout_ms));
}

void Fixture::expect_turn_right(float target_yaw, float speed, uint32_t timeout_ms)
{
    EXPECT_CALL(_mock_movementController, startTurningInPlace(target_yaw, speed, timeout_ms));
}

void Fixture::expect_stop_both_motors()
{
    EXPECT_CALL(_mock_movementController, stopBothMotors());
}

void Fixture::expect_update_movement()
{
    EXPECT_CALL(_mock_movementController, updateMovement());
}

void Fixture::expect_target_reached()
{
    EXPECT_CALL(_mock_movementController, isMoving()).WillOnce(Return(false));
    expect_enter_low_power_mode();
}

void Fixture::expect_target_not_reached()
{
    EXPECT_CALL(_mock_movementController, isMoving()).WillOnce(Return(true));
}

void Fixture::expect_radio_initialization()
{
    EXPECT_CALL(_mock_RF24, _begin()).WillOnce(Return(true));
    EXPECT_CALL(_mock_RF24, _setPALevel(PA_LEVEL, true));
    EXPECT_CALL(_mock_RF24, _setDataRate(DATA_RATE)).WillOnce(Return(true));
    EXPECT_CALL(_mock_RF24, _setAddressWidth(5));
    /* RadioController now sets payload size to CommandPacket (32 bytes) to support both
     * EventPacket (2 bytes) and CommandPacket (32 bytes) formats. */
    EXPECT_CALL(_mock_RF24, _setPayloadSize(sizeof(CommandPacket)));
    EXPECT_CALL(_mock_RF24, _openWritingPipe(::testing::_))
            .WillOnce(::testing::Invoke([this](const uint8_t *ptr) {
                for (int i = 0; i < 5; ++i) {
                    ASSERT_EQ(ptr[i], test_writing_addr[i])
                            << "Mismatch at writing address byte " << i;
                }
            }));
    EXPECT_CALL(_mock_RF24, _openReadingPipe(1, ::testing::_))
            .WillOnce(::testing::Invoke([this](uint8_t, const uint8_t *ptr) {
                for (int i = 0; i < 5; ++i) {
                    ASSERT_EQ(ptr[i], test_reading_addr[i])
                            << "Mismatch at reading address byte " << i;
                }
            }));
    EXPECT_CALL(_mock_RF24, _powerUp());
    EXPECT_CALL(_mock_RF24, _startListening());
}

void Fixture::radio_receive_event(Event *evt)
{
    EXPECT_CALL(_mock_RF24, _available(_)).WillOnce(Return(evt ? true : false));
    if (evt)
        EXPECT_CALL(_mock_RF24, _read(_, sizeof(CommandPacket)))
                .WillOnce([evt](void *buf, uint8_t len) {
                    std::cout << "_read called with buf=" << buf << " len=" << (int)len
                              << std::endl;
                    if (evt) {
                        CommandPacket packet{};
                        packet.count = 1;
                        packet.commands[0] = static_cast<uint8_t>(evt->getPayload());
                        /* Clear buffer and copy CommandPacket at the beginning. */
                        memset(buf, 0, len);
                        memcpy(buf, &packet, sizeof(packet));
                    }
                });
}

void Fixture::radio_receive_raw(const void *data, uint8_t len)
{
    EXPECT_CALL(_mock_RF24, _available(_)).WillOnce(Return(true));
    /* RadioController now reads 32 bytes to support both EventPacket and CommandPacket. */
    EXPECT_CALL(_mock_RF24, _read(_, sizeof(CommandPacket)))
            .WillOnce([data, len](void *buf, uint8_t readLen) {
                ASSERT_EQ(readLen, sizeof(CommandPacket));
                /* Copy the provided data and zero-pad the rest. */
                memset(buf, 0, readLen);
                memcpy(buf, data, (len < readLen) ? len : readLen);
            });
}

void Fixture::expect_transmit_event(Event evt)
{
    EventPacket packet;

    evt.toPacket(packet);
    auto do_check = [&packet](const void *buf, size_t len) {
        ASSERT_EQ(len, Event::packetSize());
        ASSERT_EQ(memcmp(buf, &packet, Event::packetSize()), 0);
    };
    EXPECT_CALL(_mock_RF24, _write(NotNull(), Event::packetSize()))
            .WillOnce(DoAll(Invoke(do_check), Return(true)));
}
void Fixture::expect_process_radio_no_event()
{
    radio_receive_event(nullptr);
}

void Fixture::RxRadioEvent(Event evt)
{
    if (eventQueue.IsFull()) {
        std::cout << "RadioController::SendEvent: Queue is full." << std::endl;
        return;
    }
    eventQueue.enqueue(evt);
}

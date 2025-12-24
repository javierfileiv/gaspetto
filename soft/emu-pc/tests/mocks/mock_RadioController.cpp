#include "mock_RadioController.h"

#include "Event.h"
#include "config_radio.h"

#include <cstring>
#include <gmock/gmock.h>

using testing::_;
using testing::Invoke;
using testing::Return;

void MockRadioController::expect_radio_initialization()
{
    EXPECT_CALL(_mock_RF24, _begin()).WillOnce(Return(true));
    EXPECT_CALL(_mock_RF24, _setPALevel(PA_LEVEL, true));
    EXPECT_CALL(_mock_RF24, _setDataRate(DATA_RATE)).WillOnce(Return(true));
    EXPECT_CALL(_mock_RF24, _setPayloadSize(Event::packetSize()));
    EXPECT_CALL(_mock_RF24, _openWritingPipe(::testing::_))
            .WillOnce(::testing::Invoke([this](const uint8_t *ptr) {
                for (int i = 0; i < 5; ++i) {
                    ASSERT_EQ(ptr[i], test_writing_ptr[i])
                            << "Mismatch at writing address byte " << i;
                }
            }));
    EXPECT_CALL(_mock_RF24, _openReadingPipe(1, ::testing::_))
            .WillOnce(::testing::Invoke([this](uint8_t, const uint8_t *ptr) {
                for (int i = 0; i < 5; ++i) {
                    ASSERT_EQ(ptr[i], test_reading_ptr[i])
                            << "Mismatch at reading address byte " << i;
                }
            }));
    EXPECT_CALL(_mock_RF24, _powerUp());
    EXPECT_CALL(_mock_RF24, _startListening());
}

void MockRadioController::expect_receive_event(Event *evt)
{
    EXPECT_CALL(_mock_RF24, _available(_)).WillOnce(Return(evt ? true : false));
    if (evt)
        EXPECT_CALL(_mock_RF24, _read(_, Event::packetSize()))
                .WillOnce([evt](void *buf, uint8_t len) {
                    std::cout << "_read called with buf=" << buf << " len=" << (int)len
                              << std::endl;
                    if (evt) {
                        EventPacket packet;
                        evt->toPacket(packet);
                        std::memcpy(buf, &packet, len);
                    }
                });
}

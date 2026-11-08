#include "mock_RadioController.h"

#include "GenericEvent.h"
#include "config_radio.h"

#include <cstring>
#include <gmock/gmock.h>

using testing::Return;
using testing::_;
using testing::Invoke;

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

#pragma once
#include "RadioController.h"
#include "mock_RF24.h"

#include <gmock/gmock.h>

class MockRadioController : public RadioController {
public:
    MockRadioController(EventQueue *queue, const uint8_t writing_addr[5],
                        const uint8_t reading_addr[5], testing::StrictMock<MockRF24> &mock_RF24)
            : test_writing_ptr(writing_addr)
            , test_reading_ptr(reading_addr)
            , RadioController(mock_RF24, queue, writing_addr, reading_addr)
            , _mock_RF24(mock_RF24)
            , _radio_queue(*queue)
    {
    }

    MockRadioController(const MockRadioController &) = delete;
    MockRadioController &operator=(const MockRadioController &) = delete;

    void expect_radio_initialization();
    void expect_receive_event(Event *evt = nullptr);

private:
    testing::StrictMock<MockRF24> &_mock_RF24;
    EventQueue &_radio_queue;
    const uint8_t *test_writing_ptr;
    const uint8_t *test_reading_ptr;
};

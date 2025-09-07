#pragma once
#include "RF24.h"
#include "mock_base.h"

#include <gmock/gmock.h>

class MockRF24 : public MockBase<MockRF24>, public RF24 {
public:
    MOCK_METHOD(bool, _begin, ());
    MOCK_METHOD(bool, _begin, (rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin));
    MOCK_METHOD(void, _startListening, ());
    MOCK_METHOD(void, _stopListening, ());
    MOCK_METHOD(bool, _available, ());
    MOCK_METHOD(bool, _available, (uint8_t * pipe_num));
    MOCK_METHOD(bool, _write, (const void *buf, uint8_t len));
    MOCK_METHOD(bool, _write, (const void *buf, uint8_t len, bool multicast));
    MOCK_METHOD(void, _read, (void *buf, uint8_t len));
    MOCK_METHOD(void, _openWritingPipe, (const uint8_t *address));
    MOCK_METHOD(void, _openReadingPipe, (uint8_t number, const uint8_t *address));
    MOCK_METHOD(void, _setPALevel, (uint8_t level, bool lnaEnable));
    MOCK_METHOD(bool, _setDataRate, (rf24_datarate_e speed));
    MOCK_METHOD(void, _setPayloadSize, (uint8_t size));
    MOCK_METHOD(void, _setChannel, (uint8_t channel));
    MOCK_METHOD(void, _powerUp, ());
    MOCK_METHOD(void, _powerDown, ());
    MOCK_METHOD(void, _setAddressWidth, (uint8_t width));

    bool begin(void) override;
    void startListening(void) override;
    void stopListening(void) override;
    bool available(uint8_t *pipe_num) override;
    void read(void *buf, uint8_t len) override;
    bool write(const void *buf, uint8_t len) override;
    void openWritingPipe(const uint8_t *address) override;
    void openReadingPipe(uint8_t number, const uint8_t *address) override;
    void setPALevel(uint8_t level, bool lnaEnable = true) override;
    void powerUp(void) override;
    bool setDataRate(rf24_datarate_e speed) override;
    void setPayloadSize(uint8_t size) override;
    void simulateReceivedPacket(uint8_t pipe, const void *data, uint8_t len) override;
    void simulateFailedTransmission() override;
    void simulateSuccessfulTransmission() override;
    void setAddressWidth(uint8_t width) override;

public:
    MockRF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed = 10000000);
    MockRF24(uint32_t spi_speed = 10000000);
    virtual ~MockRF24();
};

#include "mock_RF24.h"

#include <cstring>

static uint8_t g_payload[32];
static bool g_data_available = false;

MockRF24::MockRF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed)
        : MockBase<MockRF24>()
        , RF24(_cepin, _cspin, _spi_speed)
{
    set_instance(this);
}

MockRF24::MockRF24(uint32_t spi_speed)
        : MockBase<MockRF24>()
        , RF24(spi_speed)
{
    set_instance(this);
}

MockRF24::~MockRF24()
{
    clear_instance(this);
}

bool MockRF24::begin(void)
{
    auto mock = MockRF24::get_instance();

    return mock->_begin();
}

void MockRF24::startListening(void)
{
    auto mock = MockRF24::get_instance();

    mock->_startListening();
}
void MockRF24::stopListening(void)
{
    auto mock = MockRF24::get_instance();

    mock->_stopListening();
}

bool MockRF24::available()
{
    auto mock = MockRF24::get_instance();

    return mock->_available();
}

bool MockRF24::available(uint8_t *pipe_num)
{
    auto mock = MockRF24::get_instance();

    return mock->_available(pipe_num);
}

void MockRF24::read(void *buf, uint8_t len)
{
    auto mock = MockRF24::get_instance();

    mock->_read(buf, len);
    if (g_data_available) {
        std::memcpy(buf, g_payload, len > 32 ? 32 : len);
        g_data_available = false;
    }
}

bool MockRF24::write(const void *buf, uint8_t len)
{
    auto mock = MockRF24::get_instance();

    return mock->_write(buf, len);
}

void MockRF24::openWritingPipe(const uint8_t *address)
{
    auto mock = MockRF24::get_instance();

    mock->_openWritingPipe(address);
}

void MockRF24::openReadingPipe(uint8_t number, const uint8_t *address)
{
    auto mock = MockRF24::get_instance();

    mock->_openReadingPipe(number, address);
}

void MockRF24::setPALevel(uint8_t level, bool lnaEnable)
{
    auto mock = MockRF24::get_instance();

    mock->_setPALevel(level, lnaEnable);
}

bool MockRF24::setDataRate(rf24_datarate_e speed)
{
    auto mock = MockRF24::get_instance();

    return mock->_setDataRate(speed);
}

void MockRF24::setPayloadSize(uint8_t size)
{
    auto mock = MockRF24::get_instance();

    mock->_setPayloadSize(size);
}

void MockRF24::powerUp(void)
{
    auto mock = MockRF24::get_instance();

    mock->_powerUp();
}

void MockRF24::setRetries(unsigned char delay, unsigned char count)
{
    auto mock = MockRF24::get_instance();

    mock->_setRetries(delay, count);
}

void MockRF24::setAutoAck(bool enable)
{
    auto mock = MockRF24::get_instance();

    mock->_setAutoAck(enable);
}

void MockRF24::simulateReceivedPacket(uint8_t, const void *data, uint8_t len)
{
    std::memcpy(g_payload, data, len > 32 ? 32 : len);
    g_data_available = true;
}

void MockRF24::simulateFailedTransmission()
{
}

void MockRF24::simulateSuccessfulTransmission()
{
}

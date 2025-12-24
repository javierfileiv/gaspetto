/* filepath: /home/fixp/sourceCode/gaspetto/soft/emu-pc/arduino_framework/RF24.cpp */
#include "RF24.h"

#include <cstring>
#include <iostream>

/* Minimal state for emulation. */
static uint8_t g_payload[32];
static bool g_data_available = false;

namespace
{
void rf24_log([[maybe_unused]] const std::string &msg)
{
#if NRF_LOG
    std::cout << msg << '\n';
#endif
}
[[maybe_unused]] void rf24_log_noendl([[maybe_unused]] const std::string &msg)
{
#if NRF_LOG
    std::cout << msg;
#endif
}
}

RF24::RF24(rf24_gpio_pin_t cepin, rf24_gpio_pin_t cspin, uint32_t spi_speed)
{
    (void)cepin;
    (void)cspin;
    (void)spi_speed;
}

RF24::RF24(uint32_t spi_speed)
{
    (void)spi_speed;
}

bool RF24::begin(void)
{
    return true;
}

void RF24::setPALevel(uint8_t level, bool lnaEnable)
{
    (void)level;
    (void)lnaEnable;
}

bool RF24::setDataRate(rf24_datarate_e speed)
{
    (void)speed;
    return true;
}

void RF24::setPayloadSize(uint8_t size)
{
    (void)size;
}

void RF24::openWritingPipe(const uint8_t *address)
{
    (void)address;
}

void RF24::openReadingPipe(uint8_t number, const uint8_t *address)
{
    (void)number;
    (void)address;
}

void RF24::printDetails(void)
{
}

void RF24::printPrettyDetails(void)
{
}

void RF24::powerUp(void)
{
}

void RF24::startListening(void)
{
}

void RF24::stopListening(void)
{
}

bool RF24::available(uint8_t *pipe_num)
{
    if (pipe_num)
        *pipe_num = 0;
    return g_data_available;
}
void RF24::read(void *buf, uint8_t len)
{
    if (g_data_available) {
        std::memcpy(buf, g_payload, len);
        g_data_available = false;
    }
}

bool RF24::write(const void *buf, uint8_t len)
{
    (void)buf;
    (void)len;
    return true;
}

void RF24::setAddressWidth(uint8_t width)
{
    rf24_log("RF24 PC Emulation address width set to " + std::to_string(static_cast<int>(width)) +
             " bytes.");
}

void RF24::simulateReceivedPacket(uint8_t pipe, const void *data, uint8_t len)
{
    (void)pipe;
    std::memcpy(g_payload, data, len > 32 ? 32 : len);
    g_data_available = true;
}

void RF24::simulateFailedTransmission()
{
}

void RF24::simulateSuccessfulTransmission()
{
}

#include "RF24.h"

static uint8_t g_payload[32];
static bool g_data_available = false;

/* Constructors (match header exactly). */
RF24::RF24(rf24_gpio_pin_t, rf24_gpio_pin_t, uint32_t)
{
}
RF24::RF24(uint32_t)
{
}

/* Core functionality. */
bool RF24::begin(void)
{
    return true;
}
void RF24::setPALevel(uint8_t, bool)
{
}
bool RF24::setDataRate(rf24_datarate_e)
{
    return true;
}
void RF24::setPayloadSize(uint8_t)
{
}
void RF24::openWritingPipe(const uint8_t *)
{
}
void RF24::openReadingPipe(uint8_t, const uint8_t *)
{
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
bool RF24::available(uint8_t *)
{
    return false;
}
void RF24::read(void *, uint8_t)
{
}
bool RF24::write(const void *, uint8_t)
{
    return false;
}

void RF24::simulateReceivedPacket(uint8_t, const void *data, uint8_t len)
{
}
void RF24::simulateFailedTransmission()
{
}
void RF24::simulateSuccessfulTransmission()
{
}

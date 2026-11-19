#pragma once

#include "Arduino.h"
#include "RF24.h"

#include <stdint.h>

/* One bounded send for every firmware that speaks to the radio directly.
 * startWrite only loads the payload and returns before the acknowledgement
 * can be known, so txStandBy is what waits for TX_DS or MAX_RT. The timeout
 * bounds the retries the library makes after the chip gives up; the chip's
 * own retransmissions still run first, which is why a firmware that cares
 * about the worst case lowers setRetries() at init. The return value is the
 * real verdict: true only when the other side acknowledged */
inline bool radio_send(RF24 &radio, const void *data, uint8_t len, uint32_t timeoutMs)
{
    radio.stopListening();
    radio.startWrite(data, len, false);
    const bool sent = radio.txStandBy(timeoutMs, true);
    radio.startListening();
    return sent;
}

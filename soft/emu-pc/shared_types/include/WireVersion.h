#pragma once

#include <stdint.h>

/* Version of the packets exchanged over the radio. It is bumped when the
 * layout or the meaning of a packet changes, and a sender and a receiver that
 * disagree must refuse the session instead of misreading each other.
 *
 * It travels once, in the first packet of a session, never in every packet:
 * the handshake is where a mismatch costs nothing to detect */
constexpr uint8_t kWireProtocolVersion = 1;

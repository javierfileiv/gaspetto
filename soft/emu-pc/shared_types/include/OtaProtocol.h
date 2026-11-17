#pragma once

#include "WireVersion.h"

#include <stdint.h>

/* Radio transfer of a new application image. One packet fits the 32-byte
 * payload of the nRF24, and every session starts with a Begin that carries
 * the protocol version, the image length and its CRC: a mismatch is refused
 * before anything is erased */
enum class OtaKind : uint8_t {
    Begin = 1, /* data[0..1] protocol, data[2..5] length, data[6..9] crc */
    Data = 2, /* seq is the packet index, data carries 28 bytes */
    End = 3, /* the whole image was sent, verify and answer */
    Ack = 4, /* seq is the last packet or page the receiver accepted */
    Error = 5, /* data[0] is an OtaErrorCode */
};

enum class OtaErrorCode : uint8_t {
    None = 0,
    ProtocolMismatch = 1,
    TooLarge = 2,
    SequenceGap = 3,
    CrcMismatch = 4,
    FlashFailure = 5,
};

struct __attribute__((packed)) OtaPacket {
    uint8_t tag;
    uint8_t kind;
    uint16_t seq;
    uint8_t data[28];
};

static_assert(sizeof(OtaPacket) == 32, "an OTA packet must fit one nRF24 payload");

/* Seq carried by the Ack that accepts a Begin; no Data packet uses it */
constexpr uint16_t kOtaBeginAck = 0xFFFF;

/* Bytes of image carried by one Data packet */
constexpr uint32_t kOtaPayload = sizeof(OtaPacket::data);

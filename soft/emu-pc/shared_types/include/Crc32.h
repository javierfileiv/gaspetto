#pragma once

#include <stdint.h>

/* CRC-32/ISO-HDLC, the same one zlib computes on the sending side. Bitwise on
 * purpose: it costs no table, and the images it checks are a few tens of
 * kilobytes */
constexpr uint32_t kCrc32Init = 0xFFFFFFFFUL;
constexpr uint32_t kCrc32Polynomial = 0xEDB88320UL;

/* The incremental form exists for a stream that arrives by pieces: the relay
 * feeds every block as it comes off the serial port, so it never has to hold
 * the whole image nor walk it a second time */
inline uint32_t crc32Update(uint32_t crc, const uint8_t *data, uint32_t length)
{
    for (uint32_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1UL) != 0UL ? (crc >> 1) ^ kCrc32Polynomial : crc >> 1;
        }
    }

    return crc;
}

inline uint32_t crc32Finish(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFUL;
}

inline uint32_t crc32(const uint8_t *data, uint32_t length)
{
    return crc32Finish(crc32Update(kCrc32Init, data, length));
}

#pragma once

#include <stdint.h>

/* CRC-32/ISO-HDLC, the same one zlib computes on the sending side. Bitwise on
 * purpose: it costs no table, and the images it checks are a few tens of
 * kilobytes */
inline uint32_t crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1UL) != 0UL ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

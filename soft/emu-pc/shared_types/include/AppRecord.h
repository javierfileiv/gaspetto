#pragma once

#include <stdint.h>

/* The record the bootloader trusts: it says which bytes of the application
 * slot make a complete image, and whether the staging area still has to be
 * copied into place. It lives in its own sector so an interrupted update
 * leaves it, and the application it describes, untouched */
constexpr uint32_t kRecordMagic = 0x52414347UL; /* 'GCAR', little-endian */
constexpr uint16_t kRecordFormat = 1;

/* Set by the application once the staging area holds a complete, verified
 * image; cleared by the bootloader after it has copied it */
constexpr uint16_t kRecordFlagCopyPending = 0x0001;

struct __attribute__((packed)) AppRecord {
    uint32_t magic;
    uint16_t format;
    uint16_t flags;
    uint32_t length;
    uint32_t crc;
};

static_assert(sizeof(AppRecord) == 16, "the uploader writes exactly 16 bytes");

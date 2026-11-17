#pragma once

#include <stdint.h>

/* Internal flash of the F411CE, split so that an update never erases the
 * region it is running from. Sectors: 0 to 3 are 16 kB, 4 is 64 kB, 5 to 7
 * are 128 kB, and a sector is the smallest thing that can be erased */
constexpr uint32_t kBootloaderAddress = 0x08000000UL; /* sector 0, 16 kB */
constexpr uint32_t kBootloaderEnd = 0x08004000UL;

/* Validation record, alone in sector 1 so that erasing the staging area can
 * never destroy the proof that the application is good */
constexpr uint32_t kRecordAddress = 0x08004000UL;

/* Application: sectors 2 to 4, 96 kB */
constexpr uint32_t kAppBase = 0x08008000UL;
constexpr uint32_t kAppEnd = 0x08020000UL;

/* Staging: sector 5, the whole 128 kB, written by the application before the
 * bootloader copies it into place */
constexpr uint32_t kStagingAddress = 0x08020000UL;
constexpr uint32_t kStagingEnd = 0x08040000UL;
constexpr uint32_t kStagingCapacity = kStagingEnd - kStagingAddress;

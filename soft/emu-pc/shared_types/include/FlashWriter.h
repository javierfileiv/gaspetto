#pragma once

#include <stdint.h>

#ifdef ARDUINO

#include <Arduino.h>

/* Flash helpers shared by the application and the bootloader. The code that
 * calls them runs from flash, so the CPU stalls for the whole erase or
 * program; that is expected, and the caller must not need the radio or the
 * timers until the call returns */
constexpr uint32_t kFlashKey1 = 0x45670123UL;
constexpr uint32_t kFlashKey2 = 0xCDEF89ABUL;
constexpr uint32_t kFlashErrorMask = FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR |
                                     FLASH_SR_PGSERR;

inline void flashUnlock()
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0UL) {
        FLASH->KEYR = kFlashKey1;
        FLASH->KEYR = kFlashKey2;
    }
}

inline void flashLock()
{
    FLASH->CR |= FLASH_CR_LOCK;
}

inline bool flashWait()
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0UL) {
    }

    const bool ok = (FLASH->SR & kFlashErrorMask) == 0UL;
    FLASH->SR = kFlashErrorMask;
    return ok;
}

inline uint32_t flashSectorOf(uint32_t address)
{
    if (address < 0x08004000UL) {
        return 0;
    }
    if (address < 0x08008000UL) {
        return 1;
    }
    if (address < 0x0800C000UL) {
        return 2;
    }
    if (address < 0x08010000UL) {
        return 3;
    }
    if (address < 0x08020000UL) {
        return 4;
    }
    if (address < 0x08040000UL) {
        return 5;
    }
    if (address < 0x08060000UL) {
        return 6;
    }
    return 7;
}

inline bool flashEraseSector(uint32_t address)
{
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1; /* x32 parallelism */
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= (flashSectorOf(address) << FLASH_CR_SNB_Pos) & FLASH_CR_SNB;
    FLASH->CR |= FLASH_CR_STRT;

    const bool ok = flashWait();

    FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    return ok;
}

inline bool flashProgramWord(uint32_t address, uint32_t value)
{
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1;
    FLASH->CR |= FLASH_CR_PG;
    *reinterpret_cast<volatile uint32_t *>(address) = value;

    const bool ok = flashWait();

    FLASH->CR &= ~FLASH_CR_PG;
    return ok;
}

/* A flash address is a pointer on the chip */
inline const uint8_t *flashPointer(uint32_t address)
{
    return reinterpret_cast<const uint8_t *>(address);
}

#else /* ARDUINO */

#include <string.h>

/* The emulator has no flash: the record and staging sectors declared in
 * FlashLayout.h stand in for it. The address constants keep their hardware
 * values, and every access is translated to the matching buffer, so the code
 * under test runs the same 32-bit addresses it would on the chip */
inline uint8_t *flashSectorBuffer(uint32_t address, uint32_t &base, uint32_t &size)
{
    if (address >= kStagingAddress && address < kStagingEnd) {
        base = kStagingAddress;
        size = kStagingCapacity;
        return gStagingSector;
    }
    if (address >= kRecordAddress && address < kRecordAddress + kRecordSectorSize) {
        base = kRecordAddress;
        size = kRecordSectorSize;
        return gRecordSector;
    }

    base = 0;
    size = 0;
    return nullptr;
}

inline const uint8_t *flashPointer(uint32_t address)
{
    uint32_t base = 0;
    uint32_t size = 0;
    const uint8_t *buffer = flashSectorBuffer(address, base, size);

    return buffer != nullptr ? buffer + (address - base) : nullptr;
}

/* One flag for the tests: while it is set, the next erase or program fails,
 * so the error paths of the updater are walked without a broken chip. It
 * lives in the emulation only, never in the target build */
inline bool &flashFailureForTest()
{
    static bool failed = false;
    return failed;
}

inline void flashUnlock()
{
}

inline void flashLock()
{
}

inline bool flashEraseSector(uint32_t address)
{
    uint32_t base = 0;
    uint32_t size = 0;
    uint8_t *buffer = flashSectorBuffer(address, base, size);

    if (buffer == nullptr || flashFailureForTest()) {
        return false;
    }

    memset(buffer, 0xFF, size);
    return true;
}

inline bool flashProgramWord(uint32_t address, uint32_t value)
{
    uint32_t base = 0;
    uint32_t size = 0;
    uint8_t *buffer = flashSectorBuffer(address, base, size);

    if (buffer == nullptr || flashFailureForTest() || (address - base) + 4U > size) {
        return false;
    }

    memcpy(buffer + (address - base), &value, sizeof(value));
    return true;
}

#endif /* ARDUINO */

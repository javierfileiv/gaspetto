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

#else /* ARDUINO */

/* The emulator and the unit tests have no flash to write: the calls exist so
 * the receiver compiles, and they fail so the OTA path is never faked */
inline void flashUnlock()
{
}

inline void flashLock()
{
}

inline bool flashEraseSector(uint32_t)
{
    return false;
}

inline bool flashProgramWord(uint32_t, uint32_t)
{
    return false;
}

#endif /* ARDUINO */

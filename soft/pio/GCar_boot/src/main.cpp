/*
 * GCar bootloader.
 *
 * It owns sector 0 only. The application lives at 0x08008000 and the
 * validation record in sector 1, so an update can erase the staging area and
 * the application without ever touching this code.
 *
 * On boot it reads the record. If the record says the staging area holds a
 * complete image, it copies that image into place and clears the flag; then,
 * record and application agreed, it jumps. When it refuses to boot it says why
 * with the LED, because there is no serial port and no radio here:
 *
 *   1 blink  no image at all: the application slot is erased or its vectors
 *            are nonsense
 *   2 blinks the validation record is missing or unreadable
 *   3 blinks the record is there but the CRC does not match: the image is
 *            corrupted or was written halfway
 *   4 blinks the copy from the staging area failed: the flag stays set and
 *            the next boot tries again
 *
 * After a successful copy it blinks three times and starts the application, so
 * the end of an update is not silent. The three blinks stop; a refusal code
 * repeats forever
 *
 * The ROM DFU of the chip is untouched by any of this, so BOOT0 plus RESET
 * always brings the board back.
 */
#include "AppRecord.h"
#include "Crc32.h"
#include "FlashLayout.h"
#include "FlashWriter.h"
#include "RecordStore.h"

#include <Arduino.h>

namespace
{
/* SRAM of the F411CE, 128 kB */
constexpr uint32_t kRamBase = 0x20000000UL;
constexpr uint32_t kRamEnd = 0x20020000UL;

/* The BlackPill LED sits between 3V3 and PC13, so it lights when the pin is
 * pulled low */
constexpr uint32_t kLedPin = PC13;
constexpr int kLedOn = LOW;
constexpr int kLedOff = HIGH;
constexpr int kCodeOnMs = 120;
constexpr int kCodeOffMs = 180;
constexpr int kCodeGapMs = 900;
constexpr int kAliveFlashMs = 60;

/* The application slot is erased whole: the record gives the length but the
 * sectors are the erase unit, so anything shorter still costs all of them */
bool copyStagingToApplication(const AppRecord *record)
{
    if (!flashEraseSector(kAppBase) || !flashEraseSector(0x0800C000UL) ||
        !flashEraseSector(0x08010000UL)) {
        return false;
    }

    const uint32_t words = (record->length + 3) / 4;
    for (uint32_t index = 0; index < words; ++index) {
        const uint32_t offset = index * 4;
        uint32_t word = 0xFFFFFFFFUL;

        for (uint32_t byte = 0; byte < 4; ++byte) {
            const uint32_t at = offset + byte;
            if (at < record->length) {
                const uint8_t value = *reinterpret_cast<const uint8_t *>(kStagingAddress + at);
                word &= ~(0xFFUL << (byte * 8));
                word |= static_cast<uint32_t>(value) << (byte * 8);
            }
        }

        if (!flashProgramWord(kAppBase + offset, word)) {
            return false;
        }
    }

    return true;
}

bool recordLooksValid(const AppRecord *record)
{
    if (record->magic != kRecordMagic || record->format != kRecordFormat) {
        return false;
    }

    return record->length != 0 && record->length <= kAppEnd - kAppBase;
}

bool vectorsLookValid(const uint32_t *app)
{
    /* Both words come from the image itself, so they are checked before being
     * trusted: an erased flash reads 0xFFFFFFFF and jumping there faults */
    if (app[0] < kRamBase || app[0] > kRamEnd) {
        return false;
    }
    if ((app[1] & 1UL) == 0UL) {
        return false; /* The reset vector must be a Thumb address */
    }

    return app[1] >= kAppBase && app[1] < kAppEnd;
}

bool imageMatchesRecord(const AppRecord *record)
{
    return crc32(reinterpret_cast<const uint8_t *>(kAppBase), record->length) == record->crc;
}

void startApplication(const uint32_t *app)
{
    const uint32_t stack = app[0];
    const uint32_t entry = app[1];

    __disable_irq();

    /* The application re-runs the whole clock setup from scratch, the same way
     * it does after a reset. Leaving the PLL running as the system clock makes
     * that setup wait for a flag that will never change, so the chip has to be
     * handed over in the state a reset would leave it in */
    HAL_DeInit();
    HAL_RCC_DeInit();

    /* The bootloader's tick must not fire through the application's vector
     * table before that application has set itself up */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for (uint32_t index = 0; index < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++index) {
        NVIC->ICER[index] = 0xFFFFFFFFUL;
        NVIC->ICPR[index] = 0xFFFFFFFFUL;
    }

    SCB->VTOR = kAppBase;

    /* One instruction block on purpose: after the stack pointer moves, a C
     * epilogue would pop from the new stack, and the application's initial
     * stack is the very top of RAM, so that pop reads past the end of the
     * memory and faults before the application's first instruction runs */
    __asm volatile("msr msp, %0\n"
                   "cpsie i\n"
                   "bx %1\n"
                   :
                   : "r"(stack), "r"(entry)
                   : "memory");

    for (;;) {
    }
}

/* Three blinks then silence: the update is written and the application starts
 * right after, so the pattern must not repeat like a refusal code */
void blinkSequence(int blinks)
{
    pinMode(kLedPin, OUTPUT);

    for (int blink = 0; blink < blinks; ++blink) {
        digitalWrite(kLedPin, kLedOn);
        delay(kCodeOnMs);
        digitalWrite(kLedPin, kLedOff);
        delay(kCodeOffMs);
    }

    delay(kCodeGapMs);
}

[[noreturn]] void blinkCode(int blinks)
{
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, kLedOff);

    while (true) {
        for (int blink = 0; blink < blinks; ++blink) {
            digitalWrite(kLedPin, kLedOn);
            delay(kCodeOnMs);
            digitalWrite(kLedPin, kLedOff);
            delay(kCodeOffMs);
        }
        delay(kCodeGapMs);
    }
}
} /* namespace */

void setup()
{
    /* A short pulse on every boot proves the bootloader ran, before either the
     * application takes over or a refusal code starts blinking */
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, kLedOn);
    delay(kAliveFlashMs);
    digitalWrite(kLedPin, kLedOff);

    const AppRecord *record = reinterpret_cast<const AppRecord *>(kRecordAddress);
    if (!recordLooksValid(record)) {
        blinkCode(2);
    }

    bool updated = false;

    if ((record->flags & kRecordFlagCopyPending) != 0U) {
        const AppRecord pending = *record;
        AppRecord cleared = pending;
        bool done = false;

        cleared.flags = 0;

        flashUnlock();
        if (copyStagingToApplication(&pending)) {
            done = recordWrite(&cleared);
        }
        flashLock();

        if (!done) {
            /* The flag is still set, so the next boot retries the copy */
            blinkCode(4);
        }

        updated = done;
        record = reinterpret_cast<const AppRecord *>(kRecordAddress);
    }

    const uint32_t *app = reinterpret_cast<const uint32_t *>(kAppBase);
    if (!vectorsLookValid(app)) {
        blinkCode(1);
    }
    if (!imageMatchesRecord(record)) {
        blinkCode(3);
    }

    if (updated) {
        blinkSequence(3);
    }

    startApplication(app);
}

void loop()
{
    /* Never reached: setup() jumps to the application */
}

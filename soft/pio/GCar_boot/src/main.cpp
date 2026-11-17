/*
 * GCar bootloader.
 *
 * The application lives at 0x08008000, after these first 32 kB. This program
 * does nothing but check that an image is there, intact, and jump to it, so
 * that the image that cannot be reprogrammed over the air stays as small and as
 * simple as possible.
 *
 * When it refuses to boot it says why with the LED, because there is no serial
 * port and no radio here:
 *
 *   1 blink  no image at all: the application slot is erased or its vectors
 *            are nonsense
 *   2 blinks the validation record is missing or unreadable: the image may be
 *            fine, but nothing says which bytes it covers
 *   3 blinks the record is there but the CRC does not match: the image is
 *            corrupted or was written halfway
 *
 * That last code is the one worth provoking on a bench: flash the application
 * with a deliberately wrong CRC and the bootloader must refuse to jump.
 *
 * The ROM DFU of the chip is untouched by any of this, so BOOT0 plus RESET
 * always brings the board back.
 */
#include <Arduino.h>

namespace
{
/* Application slot: sectors 2 to 4, 96 kB after the bootloader */
constexpr uint32_t kAppBase = 0x08008000UL;
constexpr uint32_t kAppEnd = 0x08020000UL;

/* Validation record, first bytes of the free sector 5. It sits outside the
 * application slot on purpose: writing it must not erase a sector the image
 * lives in */
constexpr uint32_t kMetaAddress = 0x08020000UL;
constexpr uint32_t kMetaMagic = 0x52414347UL; /* 'GCAR', little-endian */
constexpr uint16_t kMetaFormat = 1;

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

struct __attribute__((packed)) AppMetadata {
    uint32_t magic;
    uint16_t format;
    uint16_t reserved;
    uint32_t length;
    uint32_t crc;
};

static_assert(sizeof(AppMetadata) == 16, "the uploader writes exactly 16 bytes");

/* CRC-32/ISO-HDLC, the same one zlib computes on the sending side. Bitwise on
 * purpose: 55 kB costs a few tens of milliseconds once at boot, and a table
 * would be a kilobyte of flash in the one image that must stay small */
uint32_t crc32(const uint8_t *data, uint32_t length)
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

bool recordLooksValid(const AppMetadata *meta)
{
    if (meta->magic != kMetaMagic || meta->format != kMetaFormat) {
        return false;
    }

    return meta->length != 0 && meta->length <= kAppEnd - kAppBase;
}

bool imageMatchesRecord(const AppMetadata *meta)
{
    return crc32(reinterpret_cast<const uint8_t *>(kAppBase), meta->length) == meta->crc;
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
    const uint32_t *app = reinterpret_cast<const uint32_t *>(kAppBase);
    const AppMetadata *meta = reinterpret_cast<const AppMetadata *>(kMetaAddress);

    /* A short pulse on every boot proves the bootloader ran, before either the
     * application takes over or a refusal code starts blinking */
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, kLedOn);
    delay(kAliveFlashMs);
    digitalWrite(kLedPin, kLedOff);

    if (!vectorsLookValid(app)) {
        blinkCode(1);
    }
    if (!recordLooksValid(meta)) {
        blinkCode(2);
    }
    if (!imageMatchesRecord(meta)) {
        blinkCode(3);
    }

    startApplication(app);
}

void loop()
{
    /* Never reached: setup() jumps to the application */
}

#pragma once

/* Shared recipe to enter the DFU bootloader of the STM32F411 system memory
 * (0x1FFF0000): the port re-enumerates as "STM32 BOOTLOADER" (0483:df11) for
 * dfu-util or pio upload_protocol=dfu. The hand-over details decide whether
 * the ROM stays there:
 *   - the ROM USB stack runs on interrupts, so PRIMASK must be left clear at
 *     the jump; __disable_irq() there makes enumeration fail and the ROM
 *     resets back to the application
 *   - the application releases the USB bus (soft-disconnect, then clock
 *     deinit) so the host sees a real detach and the ROM re-attaches on its
 *     own terms
 *   - every NVIC source is disabled and its pending flag cleared: the ROM
 *     remaps its own vector table and cannot serve a leftover app interrupt
 * Proven on the arduino_box_hw_test bench with a WeAct blackpill F411CE */

#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>

inline void enterDfuBootloader()
{
    Serial.flush();
    delay(500);
    /* Soft-disconnect the USB device so the host registers the detach before
     * the ROM tries to re-attach */
    USB_OTG_GlobalTypeDef *otg = reinterpret_cast<USB_OTG_GlobalTypeDef *>(USB_OTG_FS);
    USB_OTG_DeviceTypeDef *dev = reinterpret_cast<USB_OTG_DeviceTypeDef *>(
            reinterpret_cast<uint32_t>(otg) + USB_OTG_DEVICE_BASE);
    dev->DCTL |= USB_OTG_DCTL_SDIS;
    delay(300);
    /* Clocks back to HSI: the OTG peripheral loses its clock and releases
     * D+ with it, which the host also sees as a detach */
    HAL_RCC_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    /* Disarm every NVIC source and clear pending flags: the ROM cannot serve
     * a leftover app interrupt. PRIMASK stays clear for the ROM USB stack */
    for (uint8_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++i) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    __set_CONTROL(0);
    SCB->VTOR = 0x1FFF0000UL;
    __DSB();
    __ISB();
    const uint32_t *vectors = reinterpret_cast<const uint32_t *>(0x1FFF0000UL);
    __set_MSP(vectors[0]);
    typedef void (*BootEntry)(void);
    reinterpret_cast<BootEntry>(vectors[1])();
    for (;;) {
    }
}

#else

/* Host builds (emulator, unit tests) have no boot ROM: the function is a
 * no-op so shared code can call it unconditionally */
inline void enterDfuBootloader()
{
}

#endif /* ARDUINO_ARCH_STM32 */

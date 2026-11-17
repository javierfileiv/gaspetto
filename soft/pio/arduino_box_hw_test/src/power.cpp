/* Power side of the bench: the sensor rail lease every ADS read takes, and
 * the STOP mode deep sleep test */
#include "hwtest.h"
#include "pin_definitions.h"

#include <Arduino.h>
#include <stm32f4xx_hal.h>
#include <usbd_if.h>

#ifdef TEST_ADS1115
/* Raise the sensor rail when it is down and report whether this call did it,
 * so the caller drops it again only in that case: a rail someone else holds
 * - the health cycle, an enclosing scan - is never taken away. The nesting
 * needs no counter, the flag is the state */
bool sensorRailAcquire()
{
    if (gCalibrationRailsOn)
    {
        return false;
    }
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);
    delay(kRailSettleDelayMs);
    gCalibrationRailsOn = true;
    return true;
}

void sensorRailRelease(bool acquired)
{
    if (!acquired)
    {
        return;
    }
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    gCalibrationRailsOn = false;
}
#endif /* TEST_ADS1115 */

/* ========================================== */
/* Deep sleep (STOP) test, key "b"            */
/* ========================================== */

/* Set by the wake ISR: proves the EXTI path ran, not just "some pending
 * interrupt happened to restart the chip" */
static volatile bool gWakeButtonFired = false;

static void wakeButtonIsr()
{
    gWakeButtonFired = true;
}

/* STOP-mode test: the sequence production GBox will run under
 * LOW_POWER_MODE, with the clock restore that production still lacks
 * (its SystemClock_Config() is an empty placeholder on ARDUINO). Only
 * PB9 can bring the chip back: USB is dead while asleep. After the wake
 * the board returns to the rest state of the active mode and the shell
 * keeps running, so "b" can sleep again. */
void runDeepSleepTest()
{
    uint32_t sleepEntryMs                = 0;
    bool realPress                       = false;
    uint16_t spuriousWakes               = 0;
    constexpr uint16_t kMaxSpuriousWakes = 20;

    Serial.println();
    Serial.println("[stop] entering STOP mode: only the wake button (PB9) wakes the chip");
    Serial.println("[stop] the USB port drops now; it returns a few seconds after the wake");
    Serial.flush(); /* the warning must reach the host before clocks die */

    blackoutLeds();
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    /* Drop the latch too: the wake handler restores the rail from it, and
     * after a sleep the readings must start from a cold board again */
    gCalibrationRailsOn = false;

    pinMode(PIN_WAKE_BUTTON, INPUT_PULLUP);
    gWakeButtonFired = false;
    /* Interrupt mode (not event) is mandatory: from STOP only an EXTI
     * interrupt restarts the core, and PB9 to GND gives the FALLING edge */
    attachInterrupt(digitalPinToInterrupt(PIN_WAKE_BUTTON), wakeButtonIsr, FALLING);

    /* A spike puts the chip back in Run mode without anyone touching the
     * button, so go straight back to sleep instead of reporting a wake. Each
     * cycle drops and re-attaches the USB port, so every sleep shows up as a
     * Disconnected/Connected pair in the terminal. */
    for (;;)
    {
        gWakeButtonFired = false;
        digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
        digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);

        /* Stability of the wake line: one LOW sample already means the line drifts
         * or picks up noise, which turns into a spurious falling edge once asleep */
        const uint16_t kLineSamples = 100;
        uint16_t highSamples        = 0;
        for (uint16_t i = 0; i < kLineSamples; ++i)
        {
            highSamples += (digitalRead(PIN_WAKE_BUTTON) == HIGH) ? 1U : 0U;
            delayMicroseconds(500);
        }
        Serial.print("[stop] wake line samples: ");
        Serial.print(highSamples);
        Serial.print("/");
        Serial.print(kLineSamples);
        Serial.println(highSamples == kLineSamples ? " HIGH, stable" : " HIGH, UNSTABLE");
        Serial.flush(); /* the sample report must reach the host before the detach */

        sleepEntryMs = millis();

        /* Explicit soft-disconnect: without it the host keeps seeing the device
         * while the clocks are gone, so the terminal never prints Disconnected and
         * the sleep has no start marker. Same mechanism as enterDfuBootloader() */
        USB_OTG_GlobalTypeDef *otg    = reinterpret_cast<USB_OTG_GlobalTypeDef *>(USB_OTG_FS);
        USB_OTG_DeviceTypeDef *usbDev = reinterpret_cast<USB_OTG_DeviceTypeDef *>(
            reinterpret_cast<uint32_t>(otg) + USB_OTG_DEVICE_BASE);
        usbDev->DCTL |= USB_OTG_DCTL_SDIS;
        delay(50); /* let the host register the detach before the clock tree dies */

        /* Drop every pending EXTI request: HAL_PWR_EnterSTOPMode() goes straight to
         * __WFI(), so a flag left over from an earlier edge would return instantly
         * and the chip would never sleep */
        EXTI->PR = 0xFFFFFFFFUL;

        /* Captured after the detach window and reported after the wake: Stop mode
         * keeps RAM, so this value survives the sleep */
        const bool lineHighBeforeSleep = (digitalRead(PIN_WAKE_BUTTON) == HIGH);

        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

        /* --- Asleep here: HSI 16 MHz, PLL off, USB dead. The button pulled
         * PB9 to GND, the EXTI fired and the core resumes after the WFI. --- */

        /* Clock restore, the step production does not have yet: the
         * framework resume hook = configIPClock() + SystemClock_Config(),
         * the same weak variant function hw_config.c runs at boot */
        USBD_SystemClockConfigFromResume();
        HAL_ResumeTick();

        /* Tell a real press from a spike: a held button keeps the line low, while
         * a contact that just opened lets the filter capacitors recharge through
         * the pull-up. The node has 1.1 uF against about 7.8 k (the 10 k external
         * in parallel with the internal pull-up), so tau is 8.6 ms: read after
         * 50 ms, about six time constants, once the tick runs again */
        delay(50);
        realPress = (digitalRead(PIN_WAKE_BUTTON) == LOW);

        /* The host saw a dead device, which is not the same as an unplug:
         * force a clean detach/attach so the CDC port comes back either way */
        USBD_reenumerate();
        delay(2000); /* host re-enumeration time */

        /* Wake diagnostics: which port EXTI line 9 listens to, which lines were
         * armed during the sleep, and whether someone really held the button */
        Serial.print("[stop] EXTI line9 port=");
        Serial.print(static_cast<uint8_t>((SYSCFG->EXTICR[2] >> 4) & 0x0FU)); /* 0=A 1=B ... */
        Serial.print(" IMR=0x");
        Serial.print(EXTI->IMR, HEX);
        Serial.print(" FTSR=0x");
        Serial.print(EXTI->FTSR, HEX);
        Serial.print(" PWR->CSR=0x");
        Serial.println(PWR->CSR, HEX);
        Serial.print("[stop] wake line: before sleep=");
        Serial.print(lineHighBeforeSleep ? "HIGH" : "LOW");
        Serial.print(", after wake=");
        Serial.print(digitalRead(PIN_WAKE_BUTTON) == HIGH ? "HIGH" : "LOW");
        Serial.print(", button held=");
        Serial.println(realPress ? "YES" : "NO");
        Serial.flush();

        if (realPress)
        {
            break;
        }

        ++spuriousWakes;
        Serial.print("[stop] spurious wake #");
        Serial.print(spuriousWakes);
        Serial.println(": no button held, back to sleep");
        Serial.flush();

        if (spuriousWakes >= kMaxSpuriousWakes)
        {
            Serial.println("[stop] gave up after too many spurious wakes, back to the shell");
            break;
        }
    } /* end of the sleep/wake loop */

    /* LEDs usable again, then the dark frame (the rail cut dropped the
     * chip state) and the confirmation flashes */
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);
    delay(kRailSettleDelayMs);
    blackoutLeds();

    detachInterrupt(digitalPinToInterrupt(PIN_WAKE_BUTTON));

    const uint32_t awakeMs = millis() - sleepEntryMs; /* sleep time not counted */
    Serial.println(gWakeButtonFired ? "[stop] woke up by the wake button (EXTI fired)" :
                                      "[stop] woke up, but the button ISR did NOT run (?)");
    Serial.print("[stop] awake ms since sleep entry: ");
    Serial.print(awakeMs);
    Serial.println(" (millis does not count the sleeping time)");

    /* Proof independent from USB: three green flashes on all LEDs */
    for (uint8_t flash = 0; flash < 3; ++flash)
    {
        for (uint16_t led = 0; led < kApaPixelCount; ++led)
        {
            setOneLed(static_cast<uint8_t>(led), 0, 100, 0);
        }
        blackoutLeds();
        delay(150);
    }

    /* Hand the board back with both rails down. Every reading takes its own
     * rail now, so the mode has no rest state to restore: leaving one up
     * would only warm the board before the next measurement */
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    Serial.println("[stop] test done, back in the shell ('b' sleeps again)");
}

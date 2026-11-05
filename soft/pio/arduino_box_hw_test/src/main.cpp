#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <config_radio.h>

#ifdef TEST_ADS1115
#include <Adafruit_ADS1X15.h>
#endif
#include "CommandPacket.h"
#include "dfu_bootloader.h"
#include "hwtest.h"
#include "i2c3_bitbang.h"
#include "piece_thresholds.h"
#include "pin_definitions.h"

#include <stm32f4xx_hal.h>
#include <usbd_if.h>

/* Initialize radio object */
RF24 radio(NRF24_CE, NRF24_CSN);

/* I2C1 uses the global Wire instance */
TwoWire &i2c1 = Wire;

constexpr uint16_t kRailSettleDelayMs       = 250;
constexpr uint16_t kCalibrationScanPeriodMs = 1000;

#ifdef TEST_ADS1115

TestMode gMode           = TestMode::Health;
bool gCalibrationRailsOn = false;
/* One test run is pending, set by 's' from any mode. False at boot: the
 * board waits for a key instead of running the health cycle on its own */
bool gRunTestRequested = false;

void enterMode(TestMode mode)
{
    if (gMode == mode)
    {
        gRunTestRequested = true; /* already active: re-run this mode test */
        Serial.println("\n-> test already active, running it ('?' status)");
        return;
    }

    gMode = mode;
    if (gMode == TestMode::Calibration)
    {
        resetCalibrationStats();
        /* Force the rail state right away so the first scan sees powered
         * sensors regardless of where the health cycle was interrupted */
        digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
        digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);
        gCalibrationRailsOn = true;
        delay(kRailSettleDelayMs);
        Serial.println("\n-> mode: calibration ('s' scan, 't' gaps, '?' status)");
    }
    else
    {
        digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
        digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
        gCalibrationRailsOn = false;
        Serial.println("\n-> mode: health ('?' for status)");
    }
}

/* ========================================== */
/* ADS1115 TEST */
/* ========================================== */

void runI2CTests()
{
    Serial.println("\n--- ADS1115 I2C TEST (3V3 rail ON) ---");

    for (uint8_t i = 0; i < kAdsCount; ++i)
    {
        const AdsTestEntry &entry = kAdsTests[i];

        /* Check if device is accessible */
        bool deviceFound;
        if (entry.useBitBang)
        {
            deviceFound = i2c3_bb_probe(entry.address);
        }
        else
        {
            Adafruit_ADS1115 ads;
            deviceFound = ads.begin(entry.address, entry.wire);
        }

        if (!deviceFound)
        {
            Serial.print("   [MISS] ");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.print(" (");
            Serial.print(entry.addrNote);
            Serial.println(") — not found");
            continue;
        }

        uint16_t config = 0;
        bool configOk   = readAdsConfigRegister(&entry, config);
        if (!configOk && isI2c3Entry(&entry))
        {
            recoverI2c3Bus();
            configOk = readAdsConfigRegister(&entry, config);
        }

        if (!configOk)
        {
            Serial.print("   [ERR] ");
            Serial.print(entry.busName);
            Serial.print(" 0x");
            Serial.print(entry.address, HEX);
            Serial.println(" config read failed (transaction error / short read)");
            continue;
        }

        Serial.print("          Config register: 0x");
        Serial.print(config, HEX);
        Serial.print(" (");
        Serial.print(config, BIN);
        Serial.print(") ");

        /* Compare with ADS1115 reset value (0x8583) */
        const uint16_t ADS1115_RESET_CONFIG = 0x8583;
        if (config == ADS1115_RESET_CONFIG)
        {
            Serial.println("[DEFAULT - OK]");
        }
        else
        {
            Serial.println("[CONFIGURED/NON-DEFAULT]");
        }

        Serial.print("   [OK]  ");
        Serial.print(entry.busName);
        Serial.print(" 0x");
        Serial.print(entry.address, HEX);
        Serial.print(" (");
        Serial.print(entry.addrNote);
        Serial.println("): communication check OK (config register read)");

#ifdef TEST_ADS1115_CONVERSION_READS
        Serial.print("          conversion path test enabled (timeout-guarded, mean samples=");
        Serial.print(kAdsMeanSamples);
        Serial.println(")");
        for (uint8_t ch = 0; ch < 4; ++ch)
        {
            int16_t rawMean = 0;
            bool readOk     = readAdsSingleEndedMean(&entry, ch, rawMean);
            if (!readOk && isI2c3Entry(&entry))
            {
                recoverI2c3Bus();
                readOk = readAdsSingleEndedMean(&entry, ch, rawMean);
            }

            Serial.print("          ch");
            Serial.print(ch);
            if (readOk)
            {
                Serial.print(": conversion OK (mean raw=");
                Serial.print(rawMean);
                Serial.println(")");
            }
            else
            {
                Serial.println(": conversion TIMEOUT/FAIL");
            }
        }
#endif
    }

    Serial.println("--- ADS1115 TEST DONE ---");
}
#endif /* TEST_ADS1115 */

#define DELAY 3000

/* ========================================== */
/* SETUP */
/* ========================================== */
void setup()
{
    /* Initialize USB CDC Serial Monitor */
    Serial.begin(115200);
    delay(3000); /* Wait for Serial Monitor to open */
    Serial.println("\n\n--- HARDWARE TEST STARTED ---");

    /* The heartbeat writes PC13 from the first idle pass: without output
     * mode it toggles a floating input and the LED stays dark */
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); /* active-low: dark until the first blink */

    /* --- MOSFETs Configuration (Active-Low Logic) --- */
    /* IMPORTANT: PB14 must be OPEN DRAIN to safely handle 5V! */
    pinMode(PIN_MOSFET_5V_LEDS, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT_OPEN_DRAIN);

    /* Turn OFF all rails by default (HIGH = OFF for P-Channel MOSFETs) */
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    Serial.println("1. MOSFETs initialized and TURNED OFF.");

    /* --- APA106 LED Configuration --- */
    /* The 5V rail comes up BEFORE the first frames: the APA106 ignores
     * every frame sent while unpowered */
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);
    delay(kRailSettleDelayMs);
    Serial.println("   5V LED rail ON (settled) before APA106 init");
    ledsBegin();
    Serial.print("APA106 init: ");
    Serial.print(kApaPixelCount);
    Serial.println(" pixel(s) on PB8, first frame sent (all dark)");

    /* --- nRF24L01+ SPI Test --- */
    Serial.println("\n2. Testing nRF24L01+ SPI communication...");

    if (radio.begin())
    {
        Serial.println("   [SUCCESS] nRF24L01+ detected! SPI is working.");
        radio.setPALevel(RF24_PA_LOW); /* Set low power for testing */
        radio.printDetails();          /* Print chip info to Serial */
    }
    else
    {
        Serial.println("   [ERROR] nRF24L01+ not found!");
        Serial.println("   -> Check soldering on PA3, PA4, PA5, PA6, PA7.");
        Serial.println("   -> Check if 3.3V is reaching the nRF board.");
    }

    /* --- I2C init --- */
    Serial.println("\n3. Initializing I2C buses...");
    i2c1.begin();
    i2c1.setClock(kI2c1ClockHz);
    Serial.println("   I2C1 (SCL=PB6, SDA=PB7) @ 100kHz initialized.");

    /* Turn ON 3V3_SWITCHED FIRST so ADS1115 modules are powered */
    Serial.println("\n4. Turning ON 3V3_SWITCHED to power ADS1115 modules...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW); /* Turn ON Q2 */
    delay(kRailSettleDelayMs);                 /* Wait for modules to initialize */
    Serial.println("   [OK] 3V3_SWITCHED activated - ADS1115 modules should be powered");

    /* Initialize I2C3 bit-banging */
    Serial.println("\n5. Initializing I2C3 bit-banging (PB4=SDA, PA8=SCL)...");
    i2c3_bb_init();
    Serial.println("   [OK] I2C3 bit-banging initialized.");

    Serial.println("   NOTE: ADS1115 will be tested each cycle when 3V3 rail is ON.");

    Serial.println("   hw-test: runtime modes build");
    printUsbHelp();
    Serial.print("   mode: ");
    Serial.println(gMode == TestMode::Calibration ? "calibration" : "health");
    Serial.println("   waiting for a key ('?' for help)");
}

/* ========================================== */
/* HEALTH CYCLE (runs on demand) */
/* ========================================== */
void runHealthCycle()
{
    /* Quick double blink visual check on builtin LED. */
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }

    Serial.println("[health] cycle start ('c' = calibration, '?' = status)");

    Serial.println("-> TURNING ON 5V_SWITCHED (LED Cache)...");
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);     /* Turn ON Q1 */
    Serial.println("-> TURNING ON 3V3_SWITCHED (Sensors)...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW); /* Turn ON Q2 */

    delay(kRailSettleDelayMs);                 /* Let rails settle before I2C */

    probeI2CDevices();

#ifdef TEST_ADS1115
    /* Test ADS1115 modules while 3V3 is active */
    runI2CTests();
#endif

    /* APA106 animation must be sent while 5V LED rail is still powered. */
    runPowerOnCycleAnimation();
    runPowerOffCycleAnimation();

    reportProbeChanges();

    /* Quick double blink visual check on builtin LED. */
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }
    delayWithUsb(DELAY);

    if (gMode != TestMode::Health)
    {
        /* Mode changed while waiting: leave rails exactly as the new mode
         * set them instead of tearing the 3V3 rail down under it */
        return;
    }

    Serial.println("-> TURNING OFF 5V_SWITCHED...");
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);     /* Turn OFF Q1 */
    Serial.println("-> TURNING OFF 3V3_SWITCHED...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH); /* Turn OFF Q2 */

    Serial.println("Cycle complete. 's' reruns the test.");
    delayWithUsb(DELAY);
}

/* ========================================== */
/* MAIN LOOP */
/* ========================================== */
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

    /* Hand the board back with the rest state of the active mode: health
     * keeps the LED rail up, calibration keeps it down; the sensor rail
     * follows the calibration latch */
    digitalWrite(PIN_MOSFET_5V_LEDS, gMode == TestMode::Calibration ? HIGH : LOW);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, gCalibrationRailsOn ? LOW : HIGH);
    Serial.println("[stop] test done, back in the shell ('b' sleeps again)");
}

void loop()
{
    handleUsbCommands();

    if (gMode == TestMode::Calibration)
    {
        if (!gCalibrationRailsOn)
        {
            Serial.println("-> TURNING ON 3V3_SWITCHED (Calibration sensors)...");
            digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);
            delay(kRailSettleDelayMs);
            gCalibrationRailsOn = true;
        }

        /* Game mode: auto-capture polls the current slot and records as
         * soon as the reading settles; otherwise just USB polling between
         * steps. EMPTY is excluded from auto (it needs one deliberate
         * SPACE so a board that still holds pieces is not recorded) */
        if (gGameActive)
        {
            if (gGameAuto && kGamePieces[gGamePieceIndex] != BoxPieceId::EMPTY)
            {
                pollGameAutoCapture();
            }
            delayWithUsb(50);
            return;
        }

        /* Quick double blink visual check on builtin LED. */
        for (int i = 0; i < 2; i++)
        {
            digitalWrite(PIN_LED, LOW);
            delay(100);
            digitalWrite(PIN_LED, HIGH);
            delay(100);
        }

#ifdef TEST_ADS1115
        if (gRunTestRequested)
        {
            gRunTestRequested = false;
            runPieceCalibrationScan();
        }
#endif

        if (gMode != TestMode::Calibration)
        {
            return; /* mode changed while waiting: skip the rest delay */
        }

        delayWithUsb(kCalibrationScanPeriodMs);
        return;
    }

#ifdef TEST_ADS1115
    if (gRunTestRequested)
    {
        gRunTestRequested = false;
        runHealthCycle();
    }
#endif

    /* Quiet idle: heartbeat only, no probes, no tests between demands. */
    for (int i = 0; i < 2; i++)
    {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }
    delayWithUsb(kCalibrationScanPeriodMs);
}

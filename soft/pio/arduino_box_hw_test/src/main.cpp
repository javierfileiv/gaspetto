#include "CommandPacket.h"
#include "hwtest.h"
#include "i2c3_bitbang.h"
#include "pin_definitions.h"

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <config_radio.h>

/* Initialize radio object */
RF24 radio(NRF24_CE, NRF24_CSN);

/* Set by setup() once the chip answered: without it the send test would
 * report a missing car instead of a missing radio */
bool gRadioOk = false;

/* Radio menu (key "t"), same shape as the LED menu: the shell routes
 * every key here while it is open */
bool gRadioMenuActive = false;

/* I2C1 uses the global Wire instance */
TwoWire &i2c1 = Wire;

constexpr uint16_t kCalibrationScanPeriodMs = 1000;

#ifdef TEST_ADS1115
TestMode gMode           = TestMode::Health;
bool gCalibrationRailsOn = false;
/* One test run is pending, set by 's' from any mode. False at boot: the
 * board waits for a key instead of running the health cycle on its own */
bool gRunTestRequested = false;
#endif /* TEST_ADS1115 */

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
        /* Same line as production RadioController::begin(): the bench must
         * put the chip in the exact state the GBox will, or the link it
         * validates is not the link production will run */
        radio.setPALevel(PA_LEVEL);
        radio.setDataRate(DATA_RATE);
        radio.setAddressWidth(5);
        radio.setPayloadSize(sizeof(CommandPacket));
        radio.openWritingPipe(gcar_pipe_name); /* the GBox writes to the car */
        /* The real box layout: pipe 0 carries the program burst to _car_,
         * pipe 1 listens on _box_ where the car answers with telemetry and
         * logs ('e' opens that listening, nothing is drained outside it) */
        radio.openReadingPipe(1, gbox_pipe_name);
        radio.printDetails(); /* Print chip info to Serial */
        gRadioOk = true;
    }
    else
    {
        Serial.println("   [ERROR] nRF24L01+ not found!");
        Serial.println("   -> Check soldering on PA3, PA4, PA5, PA6, PA7.");
        Serial.println("   -> Check if 3.3V is reaching the nRF board.");
        Serial.println("   -> 'p' (send pieces) stays disabled until it answers.");
    }

    /* --- I2C init --- */
    Serial.println("\n3. Initializing I2C buses...");
    i2c1.begin();
    i2c1.setClock(kI2c1ClockHz);
    Serial.println("   I2C1 (SCL=PB6, SDA=PB7) @ 100kHz initialized.");

    /* Turn ON 3V3_SWITCHED FIRST so ADS1115 modules are powered */
    Serial.println("\n4. Turning ON 3V3_SWITCHED to power ADS1115 modules...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW); /* Turn ON Q2 */
    gCalibrationRailsOn = true;                /* the lease must not drop it from here */
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

    /* Give both rails back before waiting for a key: the boot raised them
     * for the APA106 first frames and for an ADS probe that no longer runs
     * here, and nothing measures at this point. Every measurement takes its
     * own rail now, so leaving them up would only warm the board before the
     * first reading - the offset the pulse exists to avoid */
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    gCalibrationRailsOn = false;

    Serial.println("   waiting for a key ('?' for help)");
}

/* ========================================== */
/* MAIN LOOP */
/* ========================================== */
void loop()
{
    handleUsbCommands();

    if (gMode == TestMode::Calibration)
    {
        /* No rail re-arm here: readAdsSingleEndedMean raises the sensor rail
         * for each reading and drops it again, so re-latching it would bring
         * back the warm board this change exists to avoid */

        if (gJigActive)
        {
            /* One reading per pass, and a reading raises and drops the rail
             * itself, so a pass costs the settle time: the detector sees the
             * operator within half a second, which is faster than a hand
             * placing a piece */
            pollJig();
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

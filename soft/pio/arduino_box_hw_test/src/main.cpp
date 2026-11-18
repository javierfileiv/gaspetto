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
/* 1 MHz, not the library's 10 MHz default: on this prototype the register
 * reads passed at 10 MHz while the 32-byte FIFO bursts failed about half the
 * time, so a ping was sent once every two tries. The link is reliable here at
 * 1 MHz, and 32 bytes over a slower bus costs nothing this bench waits on */
BenchRadio radio(NRF24_CE, NRF24_CSN, 1000000UL);

/* Set by setup() once the chip answered: without it the send test would
 * report a missing car instead of a missing radio */
bool radio_ok = false;

/* Radio menu (key "t"), same shape as the LED menu: the shell routes
 * every key here while it is open */
bool radio_menu_active = false;

/* I2C1 uses the global Wire instance */
TwoWire &i2c1 = Wire;

constexpr uint16_t CALIBRATION_SCAN_PERIOD_MS = 1000;

#ifdef TEST_ADS1115
TestMode test_mode        = TestMode::Health;
bool calibration_rails_on = false;
/* One test run is pending, set by 's' from any mode. False at boot: the
 * board waits for a key instead of running the health cycle on its own */
bool run_test_requested = false;
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
    delay(RAIL_SETTLE_DELAY_MS);
    Serial.println("   5V LED rail ON (settled) before APA106 init");
    leds_begin();
    Serial.print("APA106 init: ");
    Serial.print(APA_PIXEL_COUNT);
    Serial.println(" pixel(s) on PB8, first frame sent (all dark)");

    /* --- nRF24L01+ SPI Test --- */
    Serial.println("\n2. Testing nRF24L01+ SPI communication...");

    radio_begin();

    /* --- I2C init --- */
    Serial.println("\n3. Initializing I2C buses...");
    i2c1.begin();
    i2c1.setClock(I2C1_CLOCK_HZ);
    Serial.println("   I2C1 (SCL=PB6, SDA=PB7) @ 100kHz initialized.");

    /* Turn ON 3V3_SWITCHED FIRST so ADS1115 modules are powered */
    Serial.println("\n4. Turning ON 3V3_SWITCHED to power ADS1115 modules...");
    digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW); /* Turn ON Q2 */
    calibration_rails_on = true;               /* the lease must not drop it from here */
    delay(RAIL_SETTLE_DELAY_MS);               /* Wait for modules to initialize */
    Serial.println("   [OK] 3V3_SWITCHED activated - ADS1115 modules should be powered");

    /* Initialize I2C3 bit-banging */
    Serial.println("\n5. Initializing I2C3 bit-banging (PB4=SDA, PA8=SCL)...");
    i2c3_bb_init();
    Serial.println("   [OK] I2C3 bit-banging initialized.");

    Serial.println("   NOTE: ADS1115 will be tested each cycle when 3V3 rail is ON.");

    Serial.println("   hw-test: runtime modes build");
    shell_print_usb_help();
    Serial.print("   mode: ");
    Serial.println(test_mode == TestMode::Calibration ? "calibration" : "health");

    /* Give both rails back before waiting for a key: the boot raised them
     * for the APA106 first frames and for an ADS probe that no longer runs
     * here, and nothing measures at this point. Every measurement takes its
     * own rail now, so leaving them up would only warm the board before the
     * first reading - the offset the pulse exists to avoid */
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
    calibration_rails_on = false;

    Serial.println("   waiting for a key ('?' for help)");
}

/* ========================================== */
/* MAIN LOOP */
/* ========================================== */
void loop()
{
    shell_handle_usb_commands();

    if (test_mode == TestMode::Calibration)
    {
        /* No rail re-arm here: ads_read_single_ended_mean raises the sensor rail
         * for each reading and drops it again, so re-latching it would bring
         * back the warm board this change exists to avoid */

        if (jig_active)
        {
            /* One reading per pass, and a reading raises and drops the rail
             * itself, so a pass costs the settle time: the detector sees the
             * operator within half a second, which is faster than a hand
             * placing a piece */
            jig_poll();
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
        if (run_test_requested)
        {
            run_test_requested = false;
            scan_run_piece_calibration();
        }
#endif

        if (test_mode != TestMode::Calibration)
        {
            return; /* mode changed while waiting: skip the rest delay */
        }

        shell_delay_with_usb(CALIBRATION_SCAN_PERIOD_MS);
        return;
    }

#ifdef TEST_ADS1115
    if (run_test_requested)
    {
        run_test_requested = false;
        health_run_cycle();
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
    shell_delay_with_usb(CALIBRATION_SCAN_PERIOD_MS);
}

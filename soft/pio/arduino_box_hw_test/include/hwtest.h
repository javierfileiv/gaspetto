#pragma once
/* Internal plumbing of the hardware test firmware: the types, board
 * constants and functions each module exposes to the others. Not a
 * library API: everything here is specific to this bench application */
#include "CommandPacket.h"
#include "piece_thresholds.h"
#include "pin_definitions.h"

#include <Arduino.h>
#include <RF24.h>
#include <Wire.h>

/* Piece thresholds come from the shared piece_thresholds.h with their
 * static asserts: same numbers as the production firmware */

constexpr uint32_t I2C1_CLOCK_HZ = 100000;

#ifndef TEST_ADS1115_MEAN_SAMPLES
#define TEST_ADS1115_MEAN_SAMPLES 10
#endif

static_assert(TEST_ADS1115_MEAN_SAMPLES > 0, "TEST_ADS1115_MEAN_SAMPLES must be >= 1");

constexpr uint8_t ADS_MEAN_SAMPLES = TEST_ADS1115_MEAN_SAMPLES;

/* Settle time after the sensor rail changes state: a hall sensor answers in
 * microseconds, so 250 ms covers the rail's own RC plus margin. Every read
 * raises the rail for its duration and drops it again (see
 * ads_read_single_ended_mean), so this is also the pause between two readings */
constexpr uint16_t RAIL_SETTLE_DELAY_MS = 250;

/* Raise the sensor rail when it is down and report whether this call did it,
 * so the caller drops it again only in that case: a rail someone else holds
 * (the health cycle, an enclosing scan) is never taken away. Every I2C access
 * to the ADS chips needs the rail, and it must be held for the whole
 * operation - probe, reads and recovery - not only around one reading */
bool power_sensor_rail_acquire();
void power_sensor_rail_release(bool acquired);

struct CalibrationStats
{
    bool initialized;
    int16_t minRaw;
    int16_t maxRaw;
};

/* Runtime test modes, switched over the USB CDC port. Health runs the
 * connectivity cycle each pass; Calibration watches piece ADC values, with
 * the sensor rail raised per reading rather than held on for the session */
enum class TestMode : uint8_t
{
    Health,
    Calibration,
};

struct AdsTestEntry
{
    bool useBitBang; /* true = use bit-banging, false = use TwoWire */
    TwoWire *wire;   /* only used if useBitBang is false */
    uint8_t address;
    const char *busName;
    const char *addrNote;
};

/* Five ADS1115 modules of four slots each, fixed by the board wiring;
 * the shared per-slot threshold table covers the same slots */
constexpr uint8_t ADS_COUNT              = 5;
constexpr uint8_t CALIBRATION_SLOT_COUNT = ADS_COUNT * 4;

/* The packet's slot count and the number of sensors this firmware reads
 * describe the same board. The radio send test walks the packet slots and uses
 * each as an index into ADS_TESTS, so the two must stay equal, or the loop
 * would run past the table. Refuse to build rather than overrun at runtime */
static_assert(BOX_TOTAL_SLOTS == CALIBRATION_SLOT_COUNT, "packet slots must match the ADS scan");

void shell_handle_usb_commands();

/* Sliced delay so USB CDC commands are polled while the test cycles wait.
 * Rail settle delays keep the plain delay(): the electrical settling must
 * not be interrupted. Defined in shell.cpp, next to the dispatch it pumps */
void shell_delay_with_usb(uint32_t totalMs);

/* ========================================== */
/* scan.cpp - label names */
/* ========================================== */
/* Names a BoxPieceId value (0..6): the five pieces, plus the INVALID name
 * the decode returns for uncalibrated slots */
const char *scan_label_name(int index);

/* ========================================== */
/* jig.cpp - piece census and slot gain campaign */
/* ========================================== */
/* Measures every piece once in one slot, then one piece across the board to
 * get each slot's gain, and derives the per-slot rows from the two. No
 * keypress during the campaign: the reading says when a piece is in */
extern bool jig_active;

void jig_start_mode();
void jig_stop_mode();
void jig_clear_captures();
void jig_forget_slot(uint8_t slot);
void jig_print_help();
void jig_run_key(char key);
void jig_poll();
void jig_dump_captures();

/* ========================================== */
/* scan.cpp - calibration scan and slot monitor */
/* ========================================== */
extern bool monitor_active;
extern bool monitor_exit;

/* Frozen slot monitor state: the shell raises freeze_measure on SPACE
 * (unless a window is already running) and freeze_exit on any other key */
extern bool freeze_active;
extern bool freeze_measure;
extern bool freeze_measuring;
extern bool freeze_exit;

void scan_run_slot_monitor(uint8_t slotIndex);
void scan_run_frozen_slot_monitor(uint8_t slotIndex);
void scan_run_piece_calibration();

/* Walk a zone in slot order and stop at the first EMPTY; false on the first
 * INVALID. Used by the "p" send test to read the board it transmits */
bool scan_zone(uint8_t firstSlot, uint8_t endSlot, BoxBoardPieces &board, bool &sawLoopCall);

/* The one piece decode: a raw reading against this slot's compiled row.
 * Every consumer (monitors, scan, radio send) goes through it */
BoxPieceId scan_piece_for_raw(uint8_t slotIndex, int16_t raw);

/* Mode state, defined by main.cpp and read by nearly every module */
extern TestMode test_mode;
extern bool calibration_rails_on;
extern bool run_test_requested;
extern bool led_menu_active;
extern bool radio_menu_active;
/* Set by setup() once the radio answered: the send test and the stream
 * report a dead radio rather than a silent car */
extern bool radio_ok;

/* health.cpp: the periodic cycle run by the "s" key */
void health_run_cycle();

/* power.cpp: deep sleep (STOP) test driven by the "b" shell key */
void power_run_deep_sleep_test();

/* radio.cpp: scan -> build -> transmit test driven by the "p" shell key */
void radio_run_send_pieces_test();

/* radio.cpp: listen to the car stream on _box_, the "e" shell key */
void radio_run_car_stream_test();

/* radio.cpp: radio menu driven by the "t" shell key */
void radio_start_menu();
void radio_run_menu_key(char key);

/* ========================================== */
/* shell.cpp - usb key dispatch, echo and help */
/* ========================================== */
void shell_enter_mode(TestMode mode);
void shell_print_usb_help();

/* ========================================== */
/* ads.cpp - I2C bus layer, ADS1115 reads, recovery and probe history */
/* ========================================== */
extern TwoWire &i2c1;
extern RF24 radio;
extern AdsTestEntry ADS_TESTS[ADS_COUNT];
extern CalibrationStats calibration_stats[CALIBRATION_SLOT_COUNT];
extern uint32_t calibration_scan_counter;

void ads_probe_i2c_devices();
void ads_report_probe_changes();
void ads_reset_calibration_stats();
void ads_update_calibration_stats(uint8_t slotIndex, int16_t raw);
bool ads_is_i2c3_entry(const AdsTestEntry *entry);
void ads_recover_i2c3_bus();
void ads_recover_module(const AdsTestEntry *entry);
void ads_rearm_module(const AdsTestEntry *entry);
bool ads_read_config_register(const AdsTestEntry *entry, uint16_t &config);
bool ads_read_single_ended_mean(const AdsTestEntry *entry, uint8_t channel, int16_t &meanRaw);

/* ========================================== */
/* leds.cpp - APA106 status LEDs and animations */
/* ========================================== */
/* The chain is driven by the FastLED library (dedicated APA106
 * controller, RGB wire order measured on this batch) */

/* Pixels on the chain: each APA106 regenerates the signal for the next
 * one, so a pixel soldered later lights as soon as it is wired after the
 * previous DOUT. Override without editing: -D APA_LED_COUNT=n */
#ifndef APA_LED_COUNT
#define APA_LED_COUNT 3
#endif
constexpr uint16_t APA_PIXEL_COUNT = APA_LED_COUNT;

/* Global output level handed to FastLED (255 = full); override with
 * -D APA_LED_INTENSITY=n when full scale is too harsh on the bench */
#ifndef APA_LED_INTENSITY
#define APA_LED_INTENSITY 80
#endif
constexpr uint8_t APA_INTENSITY = APA_LED_INTENSITY;

/* Prepare the data pin and send one dark frame; call it AFTER the 5V
 * rail is up and settled (the APA106 ignores frames while unpowered) */
void leds_begin();
void leds_blackout();
void leds_set_one(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void leds_set_two(uint8_t ledA, uint8_t rA, uint8_t gA, uint8_t bA, uint8_t ledB, uint8_t rB,
                  uint8_t gB, uint8_t bB);
void leds_run_walk_test();
void leds_run_scan_animation_test();
void leds_run_success_animation_test();
void leds_run_build_error_animation_test();
void leds_run_empty_board_animation_test();
void leds_run_rf_error_animation_test();
void leds_run_channel_order_test(uint8_t led);
void leds_run_power_on_cycle_animation();
void leds_run_power_off_cycle_animation();
void leds_start_test_menu();
void leds_run_test_key(char key);

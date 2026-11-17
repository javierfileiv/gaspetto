/* APA106 status LEDs: the setters, canned animations and the interactive
 * test menu, on top of the FastLED APA106 controller */
#include "hwtest.h"

#include <FastLED.h>

static CRGB led_pixels[kApaPixelCount];

/* Prepare the data pin and send one dark frame. Call it AFTER the 5V
 * rail is up and settled: the APA106 ignores every frame sent while
 * unpowered */
void leds_begin()
{
    FastLED.addLeds<APA106, PIN_LED_DATA, RGB>(led_pixels, kApaPixelCount);
    FastLED.setBrightness(kApaIntensity);
    FastLED.show();
}

/* Same color on every pixel of the chain */
static void leds_fill(uint8_t r, uint8_t g, uint8_t b)
{
    fill_solid(led_pixels, kApaPixelCount, CRGB(r, g, b));
}

/* Point one pixel; arguments in RGB, same as the eye sees them */
static void leds_set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= kApaPixelCount)
    {
        return; /* an out-of-range index is dropped, the buffer cannot overflow */
    }
    led_pixels[index] = CRGB(r, g, b);
}

static void leds_show()
{
    FastLED.show();
}

constexpr uint8_t kLedState = 0;
constexpr uint8_t kLedRadio = 1;
constexpr uint8_t kLedBuild = 2;

/* Animation pacing: one constant per gesture class so the speed is tuned
 * in one place */
constexpr uint16_t kScanStepMs     = 300;  /* one pixel move in the scan bounce */
constexpr uint16_t kCascadeStepMs  = 400;  /* one pixel step of the green cascade */
constexpr uint16_t kFastBlinkOnMs  = 250;  /* lit half of the error blinks */
constexpr uint16_t kFastBlinkOffMs = 150;  /* dark half of the error blinks */
constexpr uint16_t kSlowBlinkOnMs  = 500;  /* lit half of the amber blinks */
constexpr uint16_t kSlowBlinkOffMs = 250;  /* dark half of the amber blinks */
constexpr uint16_t kHoldMs         = 2500; /* final state held before blackout */
constexpr uint16_t kChannelHoldMs  = 2500; /* pure channel check, per color */
constexpr uint16_t kWalkStepMs     = 800;  /* one lit pixel in the walk */

bool led_menu_active = false;

/* Announces one intended pixel write; an index past the chain length is
 * logged as skipped so an incomplete board still shows the full walk */
static void leds_log_pixel(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
    Serial.print("[led] LED");
    Serial.print(led);
    if (led >= kApaPixelCount)
    {
        Serial.println(" skipped (out of range)");
        return;
    }
    Serial.print(" = rgb(");
    Serial.print(r);
    Serial.print(',');
    Serial.print(g);
    Serial.print(',');
    Serial.print(b);
    Serial.println(')');
}

void leds_blackout()
{
    Serial.println("[led] blackout");
    leds_fill(0, 0, 0);
    leds_show();
}

void leds_set_one(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
    leds_log_pixel(led, r, g, b);
    leds_fill(0, 0, 0);
    leds_set_pixel(led, r, g, b);
    leds_show();
}

void leds_set_two(uint8_t ledA, uint8_t rA, uint8_t gA, uint8_t bA, uint8_t ledB, uint8_t rB,
                  uint8_t gB, uint8_t bB)
{
    leds_log_pixel(ledA, rA, gA, bA);
    leds_log_pixel(ledB, rB, gB, bB);
    leds_fill(0, 0, 0);
    leds_set_pixel(ledA, rA, gA, bA);
    leds_set_pixel(ledB, rB, gB, bB);
    leds_show();
}

/* One white pixel at a time: the position check after a fresh solder
 * joint. The first dark index names the broken link, and every pixel
 * behind it stays dark too because the chain stops forwarding there */
void leds_run_walk_test()
{
    Serial.println("[led] walk test: one white pixel at a time");
    for (uint16_t i = 0; i < kApaPixelCount; ++i)
    {
        Serial.print("[led] walk: pixel ");
        Serial.print(i);
        Serial.println(" alone (white)");
        leds_fill(0, 0, 0);
        leds_set_pixel(i, 255, 255, 255);
        leds_show();
        delay(kWalkStepMs);
    }
    leds_blackout();
}

void leds_run_scan_animation_test()
{
    /* Mirrors GBox scan: cyan bounce across 3 LEDs, 2 round-trips */
    Serial.println("[led] scan test: cyan bounce, 2 trips");
    for (int trip = 0; trip < 2; ++trip)
    {
        for (int i = 0; i < 3; ++i)
        {
            leds_set_one(static_cast<uint8_t>(i), 0, 80, 80);
            delay(kScanStepMs);
        }
        for (int i = 1; i >= 0; --i)
        {
            leds_set_one(static_cast<uint8_t>(i), 0, 80, 80);
            delay(kScanStepMs);
        }
    }
    leds_blackout();
}

void leds_run_success_animation_test()
{
    /* Mirrors GBox success: green cascade then hold all green */
    Serial.println("[led] success test: green cascade");
    leds_fill(0, 0, 0);
    for (int i = 0; i < 3; ++i)
    {
        leds_log_pixel(static_cast<uint8_t>(i), 0, 100, 0);
        leds_set_pixel(static_cast<uint16_t>(i), 0, 100, 0);
        leds_show();
        delay(kCascadeStepMs);
    }
    delay(kHoldMs);
    leds_blackout();
}

void leds_run_build_error_animation_test()
{
    /* Mirrors GBox build error: right LED red blink x3 then hold red */
    Serial.println("[led] build error test: red blink x3");
    for (int i = 0; i < 3; ++i)
    {
        leds_set_one(kLedBuild, 150, 0, 0);
        delay(kFastBlinkOnMs);
        leds_blackout();
        delay(kFastBlinkOffMs);
    }
    leds_set_one(kLedBuild, 150, 0, 0);
    delay(kHoldMs);
    leds_blackout();
}

void leds_run_empty_board_animation_test()
{
    /* Mirrors GBox empty board: right LED amber blink x2 then hold amber */
    Serial.println("[led] empty board test: amber blink x2");
    for (int i = 0; i < 2; ++i)
    {
        leds_set_one(kLedBuild, 100, 60, 0);
        delay(kSlowBlinkOnMs);
        leds_blackout();
        delay(kSlowBlinkOffMs);
    }
    leds_set_one(kLedBuild, 100, 60, 0);
    delay(kHoldMs);
    leds_blackout();
}

void leds_run_rf_error_animation_test()
{
    /* Mirrors GBox RF error: center red blink x3 then left green + center red */
    Serial.println("[led] rf error test: red blink x3, green+red hold");
    for (int i = 0; i < 3; ++i)
    {
        leds_set_one(kLedRadio, 150, 0, 0);
        delay(kFastBlinkOnMs);
        leds_blackout();
        delay(kFastBlinkOffMs);
    }
    leds_set_two(kLedState, 0, 80, 0, kLedRadio, 150, 0, 0);
    delay(kHoldMs);
    leds_blackout();
}

/* Pure channel check, one channel at a time on the given LED, long
 * enough to read by eye. The wire order lives in the addLeds template
 * call in leds_begin (RGB, measured on this batch), so a swap here means
 * that order argument is wrong */
void leds_run_channel_order_test(uint8_t led)
{
    Serial.print("[led] channel order test: pure R, G, B on LED ");
    Serial.println(led);
    Serial.println("[led] expected: RED now, then GREEN, then BLUE");
    leds_set_one(led, 255, 0, 0);
    Serial.println("[led] -> this must be RED (if not: channel order wrong)");
    delay(kChannelHoldMs);
    leds_set_one(led, 0, 255, 0);
    Serial.println("[led] -> this must be GREEN");
    delay(kChannelHoldMs);
    leds_set_one(led, 0, 0, 255);
    Serial.println("[led] -> this must be BLUE");
    delay(kChannelHoldMs);
    leds_blackout();
}

void leds_run_power_on_cycle_animation()
{
#ifdef TEST_LED_CHANNEL_ORDER
    /* Pure channel check first: the only animation that exposes a wrong
     * color order immediately */
    leds_run_channel_order_test(kLedState);
#endif
    leds_run_scan_animation_test();
    leds_run_success_animation_test();
}

void leds_run_power_off_cycle_animation()
{
    leds_run_build_error_animation_test();
    leds_run_empty_board_animation_test();
    leds_run_rf_error_animation_test();
}

/* ==========================================
 * Interactive test menu (key 'l')
 * ========================================== */

static void leds_print_test_menu()
{
    Serial.println("   [led menu] 1 walk | 2 ch LED0 | 3 ch LED1 | 4 ch LED2");
    Serial.println("              5 scan | 6 success | 7 build err | 8 empty | 9 rf err");
    Serial.println("              0 all animations | q quit");
}

/* The menu runs its tests on demand, so the 5V LED rail must be up no
 * matter which mode opened it (calibration keeps that rail off, and a
 * health cycle may have ended with it off) */
void leds_start_test_menu()
{
    Serial.println("--- LED TEST MENU ---");
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);
    delay(250);
    Serial.println("   5V LED rail ON for the menu duration");
    led_menu_active = true;
    leds_print_test_menu();
}

/* Closing gives the board back with the rail off: that is the rest state
 * of both modes (calibration requires it, a health cycle ends with it),
 * and the next 's' powers it again */
static void leds_stop_test_menu()
{
    leds_blackout();
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    led_menu_active = false;
    Serial.println("   [led menu closed, 5V rail OFF]");
}

void leds_run_test_key(char key)
{
    /* Re-arm the rail before every test: a health cycle finishing under
     * the menu would otherwise leave the tests lighting nothing */
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);

    switch (key)
    {
    case '1':
        leds_run_walk_test();
        break;
    case '2':
        leds_run_channel_order_test(0);
        break;
    case '3':
        leds_run_channel_order_test(1);
        break;
    case '4':
        leds_run_channel_order_test(2);
        break;
    case '5':
        leds_run_scan_animation_test();
        break;
    case '6':
        leds_run_success_animation_test();
        break;
    case '7':
        leds_run_build_error_animation_test();
        break;
    case '8':
        leds_run_empty_board_animation_test();
        break;
    case '9':
        leds_run_rf_error_animation_test();
        break;
    case '0':
        leds_run_power_on_cycle_animation();
        leds_run_power_off_cycle_animation();
        break;
    case 'q':
        leds_stop_test_menu();
        return;
    default:
        Serial.println("   [led menu] 1-4 = LEDs, 5-9 = animations, 0 = all, q = quit");
        return;
    }
    if (led_menu_active)
    {
        leds_print_test_menu(); /* the tests are chatty: reprint the prompt after them */
    }
}

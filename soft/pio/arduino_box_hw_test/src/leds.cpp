/* APA106 status LEDs: the setters, canned animations and the interactive
 * test menu, on top of the FastLED APA106 controller */
#include "hwtest.h"

#include <FastLED.h>

static CRGB gLeds[kApaPixelCount];

/* Prepare the data pin and send one dark frame. Call it AFTER the 5V
 * rail is up and settled: the APA106 ignores every frame sent while
 * unpowered */
void ledsBegin()
{
    FastLED.addLeds<APA106, PIN_LED_DATA, RGB>(gLeds, kApaPixelCount);
    FastLED.setBrightness(kApaIntensity);
    FastLED.show();
}

/* Same color on every pixel of the chain */
static void ledsFill(uint8_t r, uint8_t g, uint8_t b)
{
    fill_solid(gLeds, kApaPixelCount, CRGB(r, g, b));
}

/* Point one pixel; arguments in RGB, same as the eye sees them */
static void ledsSetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= kApaPixelCount)
    {
        return; /* an out-of-range index is dropped, the buffer cannot overflow */
    }
    gLeds[index] = CRGB(r, g, b);
}

static void ledsShow()
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

bool gLedMenuActive = false;

/* Announces one intended pixel write; an index past the chain length is
 * logged as skipped so an incomplete board still shows the full walk */
static void logPixel(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
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

void blackoutLeds()
{
    Serial.println("[led] blackout");
    ledsFill(0, 0, 0);
    ledsShow();
}

void setOneLed(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
    logPixel(led, r, g, b);
    ledsFill(0, 0, 0);
    ledsSetPixel(led, r, g, b);
    ledsShow();
}

void setTwoLeds(uint8_t ledA, uint8_t rA, uint8_t gA, uint8_t bA, uint8_t ledB, uint8_t rB,
                uint8_t gB, uint8_t bB)
{
    logPixel(ledA, rA, gA, bA);
    logPixel(ledB, rB, gB, bB);
    ledsFill(0, 0, 0);
    ledsSetPixel(ledA, rA, gA, bA);
    ledsSetPixel(ledB, rB, gB, bB);
    ledsShow();
}

/* One white pixel at a time: the position check after a fresh solder
 * joint. The first dark index names the broken link, and every pixel
 * behind it stays dark too because the chain stops forwarding there */
void runLedWalkTest()
{
    Serial.println("[led] walk test: one white pixel at a time");
    for (uint16_t i = 0; i < kApaPixelCount; ++i)
    {
        Serial.print("[led] walk: pixel ");
        Serial.print(i);
        Serial.println(" alone (white)");
        ledsFill(0, 0, 0);
        ledsSetPixel(i, 255, 255, 255);
        ledsShow();
        delay(kWalkStepMs);
    }
    blackoutLeds();
}

void runScanAnimationTest()
{
    /* Mirrors GBox scan: cyan bounce across 3 LEDs, 2 round-trips */
    Serial.println("[led] scan test: cyan bounce, 2 trips");
    for (int trip = 0; trip < 2; ++trip)
    {
        for (int i = 0; i < 3; ++i)
        {
            setOneLed(static_cast<uint8_t>(i), 0, 80, 80);
            delay(kScanStepMs);
        }
        for (int i = 1; i >= 0; --i)
        {
            setOneLed(static_cast<uint8_t>(i), 0, 80, 80);
            delay(kScanStepMs);
        }
    }
    blackoutLeds();
}

void runSuccessAnimationTest()
{
    /* Mirrors GBox success: green cascade then hold all green */
    Serial.println("[led] success test: green cascade");
    ledsFill(0, 0, 0);
    for (int i = 0; i < 3; ++i)
    {
        logPixel(static_cast<uint8_t>(i), 0, 100, 0);
        ledsSetPixel(static_cast<uint16_t>(i), 0, 100, 0);
        ledsShow();
        delay(kCascadeStepMs);
    }
    delay(kHoldMs);
    blackoutLeds();
}

void runBuildErrorAnimationTest()
{
    /* Mirrors GBox build error: right LED red blink x3 then hold red */
    Serial.println("[led] build error test: red blink x3");
    for (int i = 0; i < 3; ++i)
    {
        setOneLed(kLedBuild, 150, 0, 0);
        delay(kFastBlinkOnMs);
        blackoutLeds();
        delay(kFastBlinkOffMs);
    }
    setOneLed(kLedBuild, 150, 0, 0);
    delay(kHoldMs);
    blackoutLeds();
}

void runEmptyBoardAnimationTest()
{
    /* Mirrors GBox empty board: right LED amber blink x2 then hold amber */
    Serial.println("[led] empty board test: amber blink x2");
    for (int i = 0; i < 2; ++i)
    {
        setOneLed(kLedBuild, 100, 60, 0);
        delay(kSlowBlinkOnMs);
        blackoutLeds();
        delay(kSlowBlinkOffMs);
    }
    setOneLed(kLedBuild, 100, 60, 0);
    delay(kHoldMs);
    blackoutLeds();
}

void runRfErrorAnimationTest()
{
    /* Mirrors GBox RF error: center red blink x3 then left green + center red */
    Serial.println("[led] rf error test: red blink x3, green+red hold");
    for (int i = 0; i < 3; ++i)
    {
        setOneLed(kLedRadio, 150, 0, 0);
        delay(kFastBlinkOnMs);
        blackoutLeds();
        delay(kFastBlinkOffMs);
    }
    setTwoLeds(kLedState, 0, 80, 0, kLedRadio, 150, 0, 0);
    delay(kHoldMs);
    blackoutLeds();
}

/* Pure channel check, one channel at a time on the given LED, long
 * enough to read by eye. The wire order lives in the addLeds template
 * call in ledsBegin (RGB, measured on this batch), so a swap here means
 * that order argument is wrong */
void runChannelOrderTest(uint8_t led)
{
    Serial.print("[led] channel order test: pure R, G, B on LED ");
    Serial.println(led);
    Serial.println("[led] expected: RED now, then GREEN, then BLUE");
    setOneLed(led, 255, 0, 0);
    Serial.println("[led] -> this must be RED (if not: channel order wrong)");
    delay(kChannelHoldMs);
    setOneLed(led, 0, 255, 0);
    Serial.println("[led] -> this must be GREEN");
    delay(kChannelHoldMs);
    setOneLed(led, 0, 0, 255);
    Serial.println("[led] -> this must be BLUE");
    delay(kChannelHoldMs);
    blackoutLeds();
}

void runPowerOnCycleAnimation()
{
#ifdef TEST_LED_CHANNEL_ORDER
    /* Pure channel check first: the only animation that exposes a wrong
     * color order immediately */
    runChannelOrderTest(kLedState);
#endif
    runScanAnimationTest();
    runSuccessAnimationTest();
}

void runPowerOffCycleAnimation()
{
    runBuildErrorAnimationTest();
    runEmptyBoardAnimationTest();
    runRfErrorAnimationTest();
}

/* ==========================================
 * Interactive test menu (key 'l')
 * ========================================== */

static void printLedTestMenu()
{
    Serial.println("   [led menu] 1 walk | 2 ch LED0 | 3 ch LED1 | 4 ch LED2");
    Serial.println("              5 scan | 6 success | 7 build err | 8 empty | 9 rf err");
    Serial.println("              0 all animations | q quit");
}

/* The menu runs its tests on demand, so the 5V LED rail must be up no
 * matter which mode opened it (calibration keeps that rail off, and a
 * health cycle may have ended with it off) */
void startLedTestMenu()
{
    Serial.println("--- LED TEST MENU ---");
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);
    delay(250);
    Serial.println("   5V LED rail ON for the menu duration");
    gLedMenuActive = true;
    printLedTestMenu();
}

/* Closing gives the board back with the rail off: that is the rest state
 * of both modes (calibration requires it, a health cycle ends with it),
 * and the next 's' powers it again */
static void stopLedTestMenu()
{
    blackoutLeds();
    digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
    gLedMenuActive = false;
    Serial.println("   [led menu closed, 5V rail OFF]");
}

void runLedTestKey(char key)
{
    /* Re-arm the rail before every test: a health cycle finishing under
     * the menu would otherwise leave the tests lighting nothing */
    digitalWrite(PIN_MOSFET_5V_LEDS, LOW);

    switch (key)
    {
    case '1':
        runLedWalkTest();
        break;
    case '2':
        runChannelOrderTest(0);
        break;
    case '3':
        runChannelOrderTest(1);
        break;
    case '4':
        runChannelOrderTest(2);
        break;
    case '5':
        runScanAnimationTest();
        break;
    case '6':
        runSuccessAnimationTest();
        break;
    case '7':
        runBuildErrorAnimationTest();
        break;
    case '8':
        runEmptyBoardAnimationTest();
        break;
    case '9':
        runRfErrorAnimationTest();
        break;
    case '0':
        runPowerOnCycleAnimation();
        runPowerOffCycleAnimation();
        break;
    case 'q':
        stopLedTestMenu();
        return;
    default:
        Serial.println("   [led menu] 1-4 = LEDs, 5-9 = animations, 0 = all, q = quit");
        return;
    }
    if (gLedMenuActive)
    {
        printLedTestMenu(); /* the tests are chatty: reprint the prompt after them */
    }
}

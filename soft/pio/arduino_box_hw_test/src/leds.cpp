/* APA106 status LEDs: the pixel strip object, setters and canned
 * animations driven by the health cycle */
#include "hwtest.h"

#include <Adafruit_NeoPixel.h>

#ifdef TEST_LED_ANIMATIONS
constexpr uint8_t kLedState = 0;
constexpr uint8_t kLedRadio = 1;
constexpr uint8_t kLedBuild = 2;
Adafruit_NeoPixel leds(BOX_LED_COUNT, PIN_LED_DATA, NEO_GRB + NEO_KHZ800);

void blackoutLeds()
{
    leds.clear();
    leds.show();
}

void setOneLed(uint8_t led, uint8_t r, uint8_t g, uint8_t b)
{
    leds.clear();
    leds.setPixelColor(led, leds.Color(r, g, b));
    leds.show();
}

void setTwoLeds(uint8_t ledA, uint8_t rA, uint8_t gA, uint8_t bA, uint8_t ledB, uint8_t rB,
                uint8_t gB, uint8_t bB)
{
    leds.clear();
    leds.setPixelColor(ledA, leds.Color(rA, gA, bA));
    leds.setPixelColor(ledB, leds.Color(rB, gB, bB));
    leds.show();
}

void runScanAnimationTest()
{
    // Mirrors GBox scan: cyan bounce across 3 LEDs, 2 round-trips.
    for (int trip = 0; trip < 2; ++trip)
    {
        for (int i = 0; i < 3; ++i)
        {
            setOneLed(static_cast<uint8_t>(i), 0, 80, 80);
            delay(120);
        }
        for (int i = 1; i >= 0; --i)
        {
            setOneLed(static_cast<uint8_t>(i), 0, 80, 80);
            delay(120);
        }
    }
    blackoutLeds();
}

void runSuccessAnimationTest()
{
    // Mirrors GBox success: green cascade then hold all green.
    leds.clear();
    for (int i = 0; i < 3; ++i)
    {
        leds.setPixelColor(static_cast<uint16_t>(i), leds.Color(0, 100, 0));
        leds.show();
        delay(150);
    }
    delay(1200);
    blackoutLeds();
}

void runBuildErrorAnimationTest()
{
    // Mirrors GBox build error: right LED red blink x3 then hold red.
    for (int i = 0; i < 3; ++i)
    {
        setOneLed(kLedBuild, 150, 0, 0);
        delay(90);
        blackoutLeds();
        delay(45);
    }
    setOneLed(kLedBuild, 150, 0, 0);
    delay(1200);
    blackoutLeds();
}

void runEmptyBoardAnimationTest()
{
    // Mirrors GBox empty board: right LED amber blink x2 then hold amber.
    for (int i = 0; i < 2; ++i)
    {
        setOneLed(kLedBuild, 100, 60, 0);
        delay(200);
        blackoutLeds();
        delay(100);
    }
    setOneLed(kLedBuild, 100, 60, 0);
    delay(1200);
    blackoutLeds();
}

void runRfErrorAnimationTest()
{
    // Mirrors GBox RF error: center red blink x3 then left green + center red.
    for (int i = 0; i < 3; ++i)
    {
        setOneLed(kLedRadio, 150, 0, 0);
        delay(90);
        blackoutLeds();
        delay(45);
    }
    setTwoLeds(kLedState, 0, 80, 0, kLedRadio, 150, 0, 0);
    delay(1200);
    blackoutLeds();
}

void runPowerOnCycleAnimation()
{
    runScanAnimationTest();
    runSuccessAnimationTest();
}

void runPowerOffCycleAnimation()
{
    runBuildErrorAnimationTest();
    runEmptyBoardAnimationTest();
    runRfErrorAnimationTest();
}
#endif /* TEST_LED_ANIMATIONS */

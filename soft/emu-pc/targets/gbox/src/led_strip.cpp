#include "led_strip.h"

#ifdef ARDUINO

#include "pin_definitions.h"

#include <FastLED.h>

static CRGB led_strip_pixels[LED_STRIP_COUNT];

/* Call after the 5V rail is up and settled: the APA106 ignores every frame
 * sent while unpowered */
void led_strip_begin()
{
    FastLED.addLeds<APA106, PIN_LED_DATA, RGB>(led_strip_pixels, LED_STRIP_COUNT);
    FastLED.setBrightness(LED_STRIP_BRIGHTNESS);
    FastLED.show();
}

void led_strip_fill(uint8_t r, uint8_t g, uint8_t b)
{
    fill_solid(led_strip_pixels, LED_STRIP_COUNT, CRGB(r, g, b));
}

void led_strip_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= LED_STRIP_COUNT) {
        return; /* an out-of-range index is dropped, the buffer cannot overflow */
    }
    led_strip_pixels[index] = CRGB(r, g, b);
}

void led_strip_show()
{
    FastLED.show();
}

#else

/* Host builds have no strip: the calls are no-ops so shared code links */
void led_strip_begin()
{
}

void led_strip_fill(uint8_t, uint8_t, uint8_t)
{
}

void led_strip_set(uint16_t, uint8_t, uint8_t, uint8_t)
{
}

void led_strip_show()
{
}

#endif

#pragma once

#include "CommandPacket.h" /* BOX_LED_COUNT */

#include <stdint.h>

/* The APA106 chain on the box, driven through FastLED. Both the production
 * box and the bench include this header, so a colour reads the same on the
 * two and the wire order lives in one place: this batch is wired RGB, which
 * FastLED needs told explicitly or red and green come out swapped */
constexpr uint16_t LED_STRIP_COUNT = static_cast<uint16_t>(BOX_LED_COUNT);

/* Full scale is harsh on a desk; a project may override it with
 * -D LED_STRIP_BRIGHTNESS=n */
#ifndef LED_STRIP_BRIGHTNESS
#define LED_STRIP_BRIGHTNESS 80
#endif

void led_strip_begin();
void led_strip_fill(uint8_t r, uint8_t g, uint8_t b);
void led_strip_set(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void led_strip_show();

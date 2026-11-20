#pragma once

#include <stdint.h>

/* Radio tool modes: car emulation, box emulation, and quiet. They live here
 * because the OTA relay switches to the box mode and names it too */
constexpr uint8_t kModeCar = 1;
constexpr uint8_t kModeBox = 2;
constexpr uint8_t kModeQuiet = 3;

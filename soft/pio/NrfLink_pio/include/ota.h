#pragma once

#include <stdint.h>

/* The PC streams a binary image on this port: a 0x01 start byte, the image
 * length and its CRC32, then the image itself. The relay turns it into OTA
 * packets, and stays in the box mode after the verdict so the car's boot lines
 * still reach the console */
constexpr uint8_t kOtaStart = 0x01;

void runOtaStream();
void runOtaSelfTest();

/* Give the operator's mode back once the post-OTA log window is over */
void otaService();

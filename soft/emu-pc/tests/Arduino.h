#pragma once

/* Minimal stub for Arduino.h for unit tests. */
#include "Arduino_pins_pc.h"
#include "HardwareTimer.h"

#include <atomic>
#include <cstdint>

/* The emulated clock the ADS conversions move forward: a test that resets it
 * needs the same declaration the framework carries */
extern std::atomic<unsigned long> microsCounter;

/* Arduino function declarations for test linkage. */
extern "C" {
unsigned long millis(void);
unsigned long micros(void);
void SwitchToLowPowerMode(void);
void pinMode(int pin, int mode);
void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode);
void analogWrite(int pin, int value);
void digitalWrite(int pin, int value);
void analogWriteFrequency(int freq);
int digitalPinToInterrupt(int pin);
void delay(int ms);
}

#ifndef ARDUINO_H
#define ARDUINO_H
/* Minimal stub for Arduino.h for unit tests. */
#include "Arduino_pins_pc.h"
#include "HardwareTimer.h"

#include <cstdint>

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

#endif /* ARDUINO_H. */

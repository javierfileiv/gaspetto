#pragma once
#include "Arduino_pins_pc.h"
#include "Event.h"
#include "Serial.h"

#include <atomic>
#include <thread>

extern std::atomic<bool> lowPowerMode;
extern std::atomic<unsigned long> millisCounter;
extern SerialEmulator Serial;

Event getEvent(void);

extern "C" {

unsigned long millis(void);
void SwitchToLowPowerMode(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void pinMode(int pin, int mode);
void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode);
void analogWrite(int pin, int value);
int digitalPinToInterrupt(int pin);
void delay(int ms);
void analogWriteFrequency(int freq);
void printf_begin();
int digitalRead(int pin);
void digitalWrite(int pin, int value);
}

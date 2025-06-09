#pragma once
#include "Arduino_pins_pc.h"
#include "Event.h"
#include "HardwareTimer.h"
#include "Serial.h"

#include <atomic>
#include <thread>

extern std::atomic<bool> lowPowerMode;
extern std::atomic<uint32_t> millisCounter;
extern SerialEmulator Serial;

#define digitalPinToPinName(pin) pin
#define pinmap_peripheral(pin, map) nullptr
#define pinmap_function(pin, map) pin
#define STM_PIN_CHANNEL(pin) pin

Event getEvent(void);

extern "C" {

uint32_t millis(void);
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

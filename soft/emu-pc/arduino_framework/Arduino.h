#pragma once
#include "Event.h"
#include "Serial.h"

#include <atomic>
#include <thread>

extern std::atomic<bool> lowPowerMode;
extern std::atomic<unsigned long> millisCounter;
extern SerialEmulator Serial;

#define F(x) x
#define PA0 0
#define PA1 1
#define PB0 0
#define PB1 1
#define PB10 10
#define PB11 11
#define PB_15 15
#define PIN_A4 4
#define RISING 0
#define FALLING 1
#define LOW 0
#define HIGH 1
#define INPUT 0
#define INPUT_PULLUP 1
#define OUTPUT 2
#define INPUT_PULLDOWN 2
#define LOW 0
#define HIGH 1

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

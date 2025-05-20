#pragma once
#include "Event.h"
#include "Serial.h"

#include <atomic>
#include <thread>

extern std::atomic<bool> lowPowerMode;
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
#define digitalRead(pin) (pin)
#define pinMode(pin, mode) \
    do {                   \
    } while (0)
#define attachInterrupt(pin, ISR, mode) \
    do {                                \
    } while (0)
#define digitalPinToInterrupt(pin) (pin)
#define delay(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
#define digitalWrite(pin, value)
#define analogWrite(pin, value) \
    do {                        \
    } while (0)
#define analogWriteFrequency(freq) \
    do {                           \
    } while (0)
#define printf_begin() \
    do {               \
    } while (0)
unsigned long millis(void);
void enterLowPowerMode(void);
Event getEvent(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);

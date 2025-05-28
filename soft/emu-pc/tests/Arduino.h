#ifndef ARDUINO_H
#define ARDUINO_H
/* Minimal stub for Arduino.h for unit tests. */
#include "Arduino_pins_pc.h"

#include <cstdint>
#include <iostream>

/* Dummy Serial implementation. */
struct SerialType {
    void print(const char *msg)
    {
        std::cout << msg;
    }
    void print(uint32_t val)
    {
        std::cout << val;
    }
    void println(const char *msg)
    {
        std::cout << msg << std::endl;
    }
    void println(uint32_t val)
    {
        std::cout << val << std::endl;
    }
};
static SerialType Serial;

/* Dummy millis implementation. */
inline uint32_t millis()
{
    static uint32_t t = 0;
    return t += 100;
}

/* Arduino function declarations for test linkage. */
extern "C" {
void SwitchToLowPowerMode(void);
void pinMode(int pin, int mode);
void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode);
void analogWrite(int pin, int value);
void analogWriteFrequency(int freq);
int digitalPinToInterrupt(int pin);
void delay(int ms);
}

/* Dummy map implementation. */
inline int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif /* ARDUINO_H. */

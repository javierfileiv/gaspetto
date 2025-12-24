#include "Arduino.h"

#include <atomic>
#include <chrono>
#include <thread>

std::atomic<unsigned long> millisCounter{ 0 };
std::atomic<bool> lowPowerMode;

/*  Simulated millis function. */
extern "C" {
unsigned long millis(void)
{
    return millisCounter.load();
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode)
{
    (void)interruptNum;
    (void)userFunc;
    (void)mode;
}

void pinMode(int pin, int mode)
{
    (void)pin;
    (void)mode;
}

void analogWrite(int pin, int value)
{
    (void)pin;
    (void)value;
}

int digitalPinToInterrupt(int pin)
{
    return pin; /* Matches the macro definition. */
}

void analogWriteFrequency(int freq)
{
    (void)freq;
}

void printf_begin()
{
    /* Empty implementation. */
}

int digitalRead(int pin)
{
    (void)pin;
    return 1;
}

void delay(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void digitalWrite(int pin, int value)
{
    (void)pin;
    (void)value;
}

void SwitchToLowPowerMode(void)
{
    Serial.println("Entering low-power mode...\n");
#ifndef ARDUINO
    lowPowerMode.store(true);
    while (lowPowerMode.load()) {
        /* Simulate low power mode by sleeping. */
        std::this_thread::sleep_for(std::chrono::milliseconds{ 100 });
    }
#endif
}
}

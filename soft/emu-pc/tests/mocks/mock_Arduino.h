#pragma once
#include "HardwareTimer.h"
#include "mock_base.h"

#include <gmock/gmock.h>

class MockArduino : public MockBase<MockArduino> {
public:
    MOCK_METHOD(void, pinMode, (int pin, int mode));
    MOCK_METHOD(void, attachInterrupt, (int interruptNum, void (*userFunc)(void), int mode));
    MOCK_METHOD(void, digitalWrite, (int pin, int value));
    MOCK_METHOD(void, analogWrite, (int pin, int value));
    MOCK_METHOD(int, digitalPinToInterrupt, (int pin));
    MOCK_METHOD(void, delay, (int ms));
    MOCK_METHOD(void, SwitchToLowPowerMode, ());
    MOCK_METHOD(void, analogWriteFrequency, (int freq));
    MOCK_METHOD(void, setCaptureCompare,
                (uint32_t channel, uint32_t compare, TimerCompareFormat_t format));
    MOCK_METHOD(void, setPWM,
                (uint32_t channel, PinName pin, uint32_t frequency, uint32_t dutycycle));
    MOCK_METHOD(void, hw_timer_pause, ());

public:
    MockArduino();
    virtual ~MockArduino();
    void (*irq_cb_left)(void) = nullptr;
    void (*irq_cb_right)(void) = nullptr;
};

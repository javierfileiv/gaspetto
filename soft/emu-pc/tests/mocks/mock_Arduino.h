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
    MOCK_METHOD(uint32_t, millis, ());
    MOCK_METHOD(void, SwitchToLowPowerMode, ());
    MOCK_METHOD(void, analogWriteFrequency, (int freq));
    MOCK_METHOD(void, setCaptureCompare,
                (uint32_t channel, uint32_t compare, TimerCompareFormat_t format));
    MOCK_METHOD(void, setPWM,
                (uint32_t channel, PinName pin, uint32_t frequency, uint32_t dutycycle));
    MOCK_METHOD(void, pause, ());

    void execute_irq_left(int n_times = 0)
    {
        if (irq_cb_left) {
            for (int i = 0; i < n_times; ++i)
                irq_cb_left();
        }
    }

    void execute_irq_right(int n_times = 0)
    {
        if (irq_cb_right) {
            for (int i = 0; i < n_times; ++i)
                irq_cb_right();
        }
    }

public:
    MockArduino();
    virtual ~MockArduino();
    void (*irq_cb_left)(void) = nullptr;
    void (*irq_cb_right)(void) = nullptr;
};

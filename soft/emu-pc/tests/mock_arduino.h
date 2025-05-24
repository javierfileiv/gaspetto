#pragma once
#include "mock_base.h"

#include <gmock/gmock.h>

class MockArduino : public MockBase<MockArduino> {
public:
    MOCK_METHOD(void, pinMode, (int pin, int mode));
    MOCK_METHOD(void, attachInterrupt, (int interruptNum, void (*userFunc)(void), int mode));
    MOCK_METHOD(void, analogWrite, (int pin, int value));
    MOCK_METHOD(int, digitalPinToInterrupt, (int pin));
    MOCK_METHOD(void, delay, (int ms));
    MOCK_METHOD(void, SwitchToLowPowerMode, ());

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

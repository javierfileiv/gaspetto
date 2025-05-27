#include "mock_arduino.h"

#include "Arduino.h"
#include "mock_MotorController.h"

#include <atomic>
#include <gmock/gmock.h>

extern std::atomic<unsigned long> millisCounter;
extern std::atomic<bool> lowPowerMode;

MockArduino::MockArduino()
{
    set_instance(this);
}

MockArduino::~MockArduino()
{
    clear_instance(this);
}

void pinMode(int pin, int mode)
{
    auto mock = MockArduino::get_instance();

    mock->pinMode(pin, mode);
}

void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode)
{
    auto mock = MockArduino::get_instance();

    if (interruptNum == SPEED_SENSOR_LEFT_PIN) {
        mock->irq_cb_left = userFunc;
    } else if (interruptNum == SPEED_SENSOR_RIGHT_PIN) {
        mock->irq_cb_right = userFunc;
    }
    mock->attachInterrupt(interruptNum, userFunc, mode);
}

void analogWrite(int pin, int value)
{
    auto mock = MockArduino::get_instance();

    mock->analogWrite(pin, value);
}

int digitalPinToInterrupt(int pin)
{
    return pin;
}

void delay(int ms)
{
    auto mock = MockArduino::get_instance();

    mock->delay(ms);
}

void SwitchToLowPowerMode(void)
{
    auto mock = MockArduino::get_instance();

    mock->SwitchToLowPowerMode();
}

void analogWriteFrequency(int freq)
{
    auto mock = MockArduino::get_instance();

    mock->analogWriteFrequency(freq);
}

#include "HardwareTimer.h"

HardwareTimer::HardwareTimer() = default;

HardwareTimer::HardwareTimer(TIM_TypeDef *instance)
{
    (void)instance;
}

void HardwareTimer::setCaptureCompare(uint32_t channel, uint32_t compare, TimerCompareFormat format)
{
    (void)channel;
    (void)compare;
    (void)format;
}

void HardwareTimer::setPWM(uint32_t channel, PinName pin, uint32_t frequency, uint32_t dutycycle)
{
    (void)channel;
    (void)pin;
    (void)frequency;
    (void)dutycycle;
}

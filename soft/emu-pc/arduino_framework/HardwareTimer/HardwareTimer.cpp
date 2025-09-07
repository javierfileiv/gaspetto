#include "HardwareTimer.h"

HardwareTimer::HardwareTimer()
{
}

HardwareTimer::HardwareTimer(TIM_TypeDef *instance)
{
}

void HardwareTimer::setCaptureCompare(uint32_t channel, uint32_t compare,
                                      TimerCompareFormat_t format)
{
}

void HardwareTimer::setPWM(uint32_t channel, PinName pin, uint32_t frequency, uint32_t dutycycle)
{
}

void HardwareTimer::pause()
{
}

#ifndef HARDWARETIMER_H
#define HARDWARETIMER_H
#include "Arduino.h"

#include <functional>
#include <stdint.h>

typedef struct {
    uint32_t __;

} TIM_TypeDef;

#define PinName int
typedef std::function<void(void)> callback_function_t;
#define PinMap_PWM 0

typedef enum {
    PERCENT_COMPARE_FORMAT,
} TimerCompareFormat_t;

class HardwareTimer {
public:
    HardwareTimer();
    HardwareTimer(TIM_TypeDef *instance);

    void setPWM(uint32_t channel, PinName pin, uint32_t frequency, uint32_t dutycycle);
    void setCaptureCompare(uint32_t channel, uint32_t compare, TimerCompareFormat_t format);
};

#endif /* HARDWARETIMER_H */

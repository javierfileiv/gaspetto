#pragma once
#include "StateMachine.h"
#include <atomic>
#include <thread>

extern std::atomic<bool> lowPowerMode;

unsigned long millis(void);
void enterLowPowerMode(void);
Event getEvent(void);
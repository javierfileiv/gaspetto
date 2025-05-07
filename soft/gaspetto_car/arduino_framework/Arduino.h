#pragma once
#include <atomic>
#include <thread>

extern std::atomic<bool> lowPowerMode;

unsigned long millis(void);
void enterLowPowerMode(void);
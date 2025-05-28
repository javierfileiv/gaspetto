#pragma once

#include "RF24.h"

extern RF24 radio;

void loop(void);
void setup(void);
void ISR(void);

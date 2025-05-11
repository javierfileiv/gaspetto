#ifndef GLOBALS_H
#define GLOBALS_H

#include "types.h"

#include <RF24.h>
#include <nRF24L01.h>
#include <stdint.h>

extern RF24 radio;
extern ReceiveMode receiveMode;
extern TestMode testMode;
extern uint16_t testCounter;

extern const Command command_list[];

#endif // GLOBALS_H

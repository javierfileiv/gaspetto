#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "types.h"

extern RF24 radio;
extern ReceiveMode  receiveMode;
extern TestMode     testMode;
extern uint16_t     testCounter;

extern const Command command_list[];

#endif //GLOBALS_H

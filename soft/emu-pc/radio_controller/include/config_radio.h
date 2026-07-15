#pragma once

#include "Arduino.h"
#include "RF24.h"

const uint8_t gbox_pipe_name[] = "_box_";
const uint8_t gcar_pipe_name[] = "_car_";
/* Log pipe from the Box. */
const uint8_t gbox_log_pipe_name[] = "_logb";
/* Log pipe from the Car. */
const uint8_t gcar_log_pipe_name[] = "_logc";

const uint32_t PA_LEVEL = RF24_PA_LOW; /* Power Amplifier level */
const rf24_datarate_e DATA_RATE = RF24_1MBPS; /* Data rate for RF24 communication */

#ifdef ARDUINO_AVR_UNO
const uint32_t NRF24_CE = 9; /* Chip enable RF24 pin*/
const uint32_t NRF24_CSN = 10; /* Chip select RF24 pin*/
#else
#ifdef GCAR
const uint32_t NRF24_CE = PB_15; /* Chip enable RF24 pin*/
const uint32_t NRF24_CSN = PA_4; /* Chip select RF24 pin*/
#elif GBOX
const uint32_t NRF24_CE = PA8; /* Chip enable RF24 pin*/
const uint32_t NRF24_CSN = PA9; /* Chip select RF24 pin*/
#else
#warning "No radio pins defined. Using default for utests"
const uint32_t NRF24_CE = 0; /* Chip enable RF24 pin*/
const uint32_t NRF24_CSN = 0; /* Chip select RF24 pin*/
#endif
#endif

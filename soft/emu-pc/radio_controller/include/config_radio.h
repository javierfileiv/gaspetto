#ifndef CONFIG_RADIO_H
#define CONFIG_RADIO_H

#include "Arduino.h"
#include "RF24.h"

const uint8_t gaspetto_box_pipe_name[] = "_box_";
const uint8_t gaspetto_car_pipe_name[] = "_car_";

const uint32_t CE_PIN = PB15; /* Chip enable RF24 pin*/
const uint32_t CSN_PIN = PA4; /* Chip select RF24 pin*/
const uint32_t PA_LEVEL = RF24_PA_LOW; /* Power Amplifier level */
const rf24_datarate_e DATA_RATE = RF24_1MBPS; /* Data rate for RF24 communication */

#endif /* CONFIG_RADIO_H */

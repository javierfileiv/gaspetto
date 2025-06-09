#ifndef CONFIG_RADIO_H
#define CONFIG_RADIO_H

#include "Arduino.h"
#include "RF24.h"

// const uint8_t gaspetto_box_pipe_name[] = "_box_";
const uint8_t gaspetto_box_pipe_name[] = { 0x5F, 0x62, 0x6F, 0x78, 0x5F };
const uint8_t gaspetto_car_pipe_name[] = "_car_";

const rf24_datarate_e DATA_RATE = RF24_250KBPS; /* Data rate for RF24 communication */
const uint32_t PA_LEVEL = RF24_PA_LOW; /* Power Amplifier level */

#ifndef NRF_SENDER /* NRF Sender CSN PIN is on PB15 instead, that's why the #ifndef. */
const uint32_t CE_PIN = PIN_A2; /* Chip enable RF24 pin*/
const uint32_t CSN_PIN = PIN_A4; /* Chip select RF24 pin*/
#endif

#endif /* CONFIG_RADIO_H */

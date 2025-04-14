#ifndef NRF_CONFIG_H
#define NRF_CONFIG_H

#include "Arduino.h"
#include <stdint.h>

// Pines para el NRF24L01+
#define NRF_CE_PIN  PB_15
#define NRF_CSN_PIN PIN_A4

// Dirección del pipe para la comunicación
const uint64_t NRF_WRITE_ADDRESS = 0xF0F0F0F0E1LL;
const uint64_t NRF_READ_ADDRESS = 0xF0F0F0F0D2LL;

#endif
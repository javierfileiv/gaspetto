#pragma once

#include <stdint.h>

/* Piece ADC thresholds, ranges in raw ADS1115 counts at GAIN_ONE.
 * Values are placeholders until the pieces are fabricated: re-measure
 * on the physical board with the piece calibration mode of
 * soft/pio/arduino_box_hw_test, then pin the measured values here.
 * Both the production firmware and the hardware test tools include
 * this header so thresholds diverge nowhere */

#ifndef GASPETTO_ADC_THRESHOLD_FORWARD_START
#define GASPETTO_ADC_THRESHOLD_FORWARD_START 100
#endif

#ifndef GASPETTO_ADC_THRESHOLD_BACKWARD_START
#define GASPETTO_ADC_THRESHOLD_BACKWARD_START 700
#endif

#ifndef GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START
#define GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START 1300
#endif

#ifndef GASPETTO_ADC_THRESHOLD_TURN_LEFT_START
#define GASPETTO_ADC_THRESHOLD_TURN_LEFT_START 1900
#endif

#ifndef GASPETTO_ADC_THRESHOLD_LOOP_START
#define GASPETTO_ADC_THRESHOLD_LOOP_START 2500
#endif

#ifndef GASPETTO_ADC_THRESHOLD_LOOP_END
#define GASPETTO_ADC_THRESHOLD_LOOP_END 3099
#endif

#define GASPETTO_ADC_THRESHOLD_MAX 4095

static_assert(GASPETTO_ADC_THRESHOLD_FORWARD_START > 0,
              "GASPETTO_ADC_THRESHOLD_FORWARD_START must be > 0");
static_assert(GASPETTO_ADC_THRESHOLD_FORWARD_START < GASPETTO_ADC_THRESHOLD_BACKWARD_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_BACKWARD_START < GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_TURN_RIGHT_START < GASPETTO_ADC_THRESHOLD_TURN_LEFT_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_TURN_LEFT_START < GASPETTO_ADC_THRESHOLD_LOOP_START,
              "ADC thresholds must be strictly increasing");
static_assert(GASPETTO_ADC_THRESHOLD_LOOP_START <= GASPETTO_ADC_THRESHOLD_LOOP_END,
              "GASPETTO_ADC_THRESHOLD_LOOP_START must be <= LOOP_END");
static_assert(GASPETTO_ADC_THRESHOLD_LOOP_END <= GASPETTO_ADC_THRESHOLD_MAX,
              "GASPETTO_ADC_THRESHOLD_LOOP_END must be <= GASPETTO_ADC_THRESHOLD_MAX");

#pragma once

#include <Arduino.h>
#include <cstdint>
#include <stdint.h>

#define GROUP_SIZE 4
#define NUM_GROUPS 5
#define TOTAL_SENSORS (GROUP_SIZE * NUM_GROUPS)

class ReadHall {
public:
  ReadHall(uint8_t adcPins[GROUP_SIZE], uint8_t groupPins[TOTAL_SENSORS]);
  uint8_t *readHallSensors(uint8_t group);

private:
  uint8_t adcPins[GROUP_SIZE];
  uint8_t groupPins[TOTAL_SENSORS];
  const uint8_t numGroups = NUM_GROUPS;
  uint8_t sensorsPerGroup = GROUP_SIZE;
  // ADC Values array
  uint16_t adcValues[GROUP_SIZE] = {0};
};
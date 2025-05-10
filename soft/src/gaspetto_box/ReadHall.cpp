#include "ReadHall.h"
#include "Arduino.h"
#include <stdint.h>

ReadHall::ReadHall(uint8_t (&adcPins)[GROUP_SIZE],
                   uint8_t (&groupPins)[TOTAL_SENSORS]) {
  for (uint8_t i = 0; i < 4; i++) {
    this->adcPins[i] = adcPins[i];
  }
  for (uint8_t i = 0; i < 20; i++) {
    this->groupPins[i] = groupPins[i];
  }

  /*  Set GPIO pins as OUTPUT and set them HIGH (disable all sensors). */
  for (uint8_t i = 0; i < 20; i++) {
    pinMode(groupPins[i], OUTPUT);
    digitalWrite(groupPins[i], HIGH); /*  Disable sensor. */
  }

  /*  Set ADC pins as INPUT. */
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(adcPins[i], INPUT_ANALOG);
  }
}

uint16_t *ReadHall::readHallSensors(uint8_t group) {

  if (group >= numGroups) {
    Serial.println("Invalid group number");
    return nullptr; /*  Return null pointer if the group number is invalid. */
  }
  /*  Enable the current group of sensors. */
  for (uint8_t i = 0; i < sensorsPerGroup; i++) {
    /*  Enable sensor. */
    digitalWrite(groupPins[group * sensorsPerGroup + i], LOW);
  }

  /*  Small delay to allow stabilization. */
  delayMicroseconds(50);

  /*  Read ADC values for the current group. */
  for (uint8_t i = 0; i < sensorsPerGroup; i++) {
    adcValues[group * sensorsPerGroup + i] = analogRead(adcPins[i]);
  }

  /*  Disable the current group of sensors. */
  for (uint8_t i = 0; i < sensorsPerGroup; i++) {
    digitalWrite(groupPins[group * sensorsPerGroup + i],
                 HIGH); /*  Disable sensor. */
  }

  /*  Small delay before switching to the next group. */
  delayMicroseconds(50);

  return adcValues;
}

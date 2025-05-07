#include "Arduino.h"
#include "EventQueue.h"
#include "GaspettoCar.h"

void setup() {}

void loop() { gaspetto.processNextEvent(); }
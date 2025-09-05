// Command processor interface
#pragma once
#include <Arduino.h>

void commandProcessSerial();
void commandProcessToken(const String &tok);
void commandSetLastCmd(const String &tok);

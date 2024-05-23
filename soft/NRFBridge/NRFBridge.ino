#include "Timer.h"
#include "globals.h"
#include "stringstream.h"
#include "types.h"
#include "utils.h"
#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>
#include <stdint.h>
#include <time.h>

// https://github.com/bartoszbielawski/RFBridge/commits/master/src/main.cpp
// for testing
// picocom /dev/ttyUSB0 --baud 115200 --omap crcrlf --echo
//
#define VERSION "0.1.2"
#define DATA_RATE RF24_1MBPS
uint8_t address[][6] = {"1Node", "2Node"};

RF24 radio(9, 10); // NRF24L01 used SPI pins + Pin 9 and 10 on the NANO

// configure behaviour of your system here:
ReceiveMode receiveMode = ReceiveMode::Binary;
TestMode testMode = TestMode::Disabled;
rf24_pa_dbm_e defaultPower = RF24_PA_LOW;
bool openPipesByDefault = true;

Timer t;
float payload = 0.0;

void testIrq() {
  if (testMode != TestMode::Counter)
    return;
  radio.stopListening();
  Serial.print(F("Writing "));
  Serial.println(payload); // print the payload's value

  if (!radio.write(&payload, sizeof(payload)))
    Serial.println("Error writing");
  radio.startListening();
  payload += 0.01; // increment float payload
}

String command;

void setup(void) {

  command.reserve(72);
  Serial.begin(115200);
  printf_begin();
  Serial.println("INFO: RF Bridge " VERSION);
  radio.begin();
  if (!radio.begin()) {
    Serial.println(F("radio hardware not responding!"));
    while (1) {
    } // hold program in infinite loop to prevent subsequent errors
  }
  radio.setPALevel(defaultPower);
  radio.setDataRate(DATA_RATE);
  if (openPipesByDefault) {
    radio.openWritingPipe(address[0]);
    radio.openReadingPipe(1, address[1]);
  }
  radio.setPayloadSize(4);
  radio.startListening();
  radio.printPrettyDetails();
  t.every(1000, testIrq);
}

void loop(void) {
  t.update();

  switch (receiveMode) {
  case ReceiveMode::Binary:
    readDataBinary(false);
    break;
  case ReceiveMode::Hex:
    readDataHex(false);
    break;
  default:;
  }

  if (!Serial.available()) {
    return;
  }

  while (Serial.available() > 0) {
    char c = Serial.read();

    if ((c != '\n') && (c != '\r')) {
      command += c;
      continue;
    }

    if (c == '\r')
      continue;

    if (command.length() == 0)
      continue;

    StringStream commandStream(command);
    commandStream.setTimeout(0);
    commandStream.discard(5);
    String cmd = command.substring(0, 5);
    cmd.toUpperCase();
    command.toUpperCase();

    for (int i = 0;; ++i) {

      Command commandStruct = {"", nullptr};
      memcpy_P(&commandStruct, &command_list[i], sizeof(command_list[0]));
      if (commandStruct.function == nullptr) {
        Serial.print(F("CMD: Error, unknown command: "));
        Serial.println(command);
        break;
      }

      if (cmd == commandStruct.command) {
        commandStruct.function(commandStream);
        break;
      }
    }

    command = "";
  } // while available
}
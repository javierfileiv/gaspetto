#ifndef NRFCOMMUNICATOR_H
#define NRFCOMMUNICATOR_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "MessageQueue.h"
#include "NrfConfig.h"
#include "Message.h"
#include "MessageNrfTx.h" // Include para MessageNrfTx
#include "MessageNrfRx.h" // Include para MessageNrfRx

class NrfCommunicator {
public:
  NrfCommunicator(RF24& radio, MessageQueue& messageQueue);
  void processNrf(); // Maneja tanto envío como recepción
private:
  RF24& radio_;
  MessageQueue& messageQueue_;
  void sendMessage(const char* data);
  void receiveMessage();
};

#endif
#ifndef MESSAGENRF_RX_H
#define MESSAGENRF_RX_H

#include "Message.h"
#include <string>

class MessageNrfRx : public Message {
public:
  MessageNrfRx(const char* receivedData);
  void execute() override; // Procesar los datos recibidos
  MessageType getType() const override { return MessageType::NRF_RX; }
  const char* getData() const { return receivedData_; }
private:
  char receivedData_[32];
};

#endif
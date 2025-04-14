#ifndef MESSAGENRFTX_H
#define MESSAGENRFTX_H

#include "Message.h"
#include <string>

class MessageNrfTx : public Message {
public:
  MessageNrfTx(const char* dataToSend);
  void execute() override {} // No se ejecuta directamente, se envía
  MessageType getType() const override { return MessageType::NRF_TX; }
  bool shouldSend() const override { return true; }
  const char* getPayload() const override { return data_; }
private:
  char data_[32];
};

#endif
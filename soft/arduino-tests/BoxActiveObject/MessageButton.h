#ifndef MESSAGE_BUTTON_H
#define MESSAGE_BUTTON_H

#include "Message.h"
#include <stdint.h>
#include <string>

class MessageButton : public Message {
public:
  MessageButton(uint8_t pin);
  void execute() override;
  MessageType getType() const override { return MessageType::BUTTON; }
  bool shouldSend() const override { return true; } // Los botones se envían
  const char *getPayload() const override;
  uint8_t getPin() const { return pin_; }

private:
  uint8_t pin_;
  char payload_[32];
};

#endif

#ifndef MESSAGE_PRINT_H
#define MESSAGE_PRINT_H

#include "Message.h"
#include <string>

class MessagePrint : public Message {
public:
  MessagePrint(const char* text);
  void execute() override;
  MessageType getType() const override { return MessageType::PRINT; }
  const char* getText() const { return text_; }
private:
  char text_[32]; // Tamaño fijo del texto
};

#endif
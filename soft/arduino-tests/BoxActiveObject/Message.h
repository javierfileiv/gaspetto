#ifndef MESSAGE_H
#define MESSAGE_H

enum class MessageType { BUTTON, PRINT, NRF_TX, NRF_RX, PRINT_TIMED };

class Message {
public:
  virtual void execute() = 0;
  virtual ~Message() {}
  virtual MessageType getType() const = 0;
  virtual bool shouldSend() const { return false; }
  virtual const char *getPayload() const { return nullptr; }
};

#endif
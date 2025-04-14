#ifndef MESSAGEPRINTTIMED_H
#define MESSAGEPRINTTIMED_H

#include "Message.h"
#include <string>

class MessagePrintTimed : public Message {
public:
  MessagePrintTimed(const char *text, unsigned long delayMs);
  void execute() override;
  MessageType getType() const override { return MessageType::PRINT_TIMED; }
  const char *getText() const { return text_; }
  unsigned long getDelayMs() const { return delayMs_; }
  unsigned long getStartTime() const { return startTime_; }
  void setDelayedTrue(unsigned long delay) { delayMs_ = delay; }

private:
  char text_[32];
  unsigned long delayMs_;
  unsigned long startTime_ = 0;
  bool delayed_ = false;
  bool printed_ = false; // Nuevo estado para controlar la impresión única
};

#endif
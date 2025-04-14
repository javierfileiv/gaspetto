// MessageProcessor.h
#ifndef MESSAGE_PROCESSOR_H
#define MESSAGE_PROCESSOR_H

#include "MessageQueue.h"

class MessageProcessor {
public:
  MessageProcessor(MessageQueue& messageQueue);
  void processMessages();
private:
  MessageQueue& messageQueue_;
};

#endif
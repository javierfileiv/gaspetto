#ifndef MESSAGESCHEDULER_H
#define MESSAGESCHEDULER_H

#include "MessageQueue.h"
#include "Message.h"

class MessageScheduler {
public:
  static const int MAX_SCHEDULED_MESSAGES = 5; // Define la capacidad máxima
  MessageScheduler(MessageQueue& messageQueue);
  bool scheduleMessage(Message* message, unsigned long delayMs);
  void processScheduledMessages();

private:
  MessageQueue& messageQueue_;
  struct ScheduledItem {
    unsigned long triggerTime;
    Message* message;
    bool active;
  };
  ScheduledItem scheduledMessages_[MAX_SCHEDULED_MESSAGES];
  int count_ = 0; // Keep track of the number of active scheduled messages
};

#endif
#include "MessageScheduler.h"
#include <Arduino.h>

MessageScheduler::MessageScheduler(MessageQueue& messageQueue)
    : messageQueue_(messageQueue), count_(0) {
  for (int i = 0; i < MAX_SCHEDULED_MESSAGES; ++i) {
    scheduledMessages_[i].active = false;
  }
}

bool MessageScheduler::scheduleMessage(Message* message, unsigned long delayMs) {
  unsigned long triggerTime = millis() + delayMs;

  if (count_ < MAX_SCHEDULED_MESSAGES) {
    int insertIndex = 0;
    // Find the correct position to insert the new message to maintain sorted order
    while (insertIndex < count_ && scheduledMessages_[insertIndex].triggerTime <= triggerTime) {
      insertIndex++;
    }

    // Shift existing messages to make space for the new one
    for (int i = count_; i > insertIndex; --i) {
      scheduledMessages_[i] = scheduledMessages_[i - 1];
    }

    // Insert the new message
    scheduledMessages_[insertIndex].triggerTime = triggerTime;
    scheduledMessages_[insertIndex].message = message;
    scheduledMessages_[insertIndex].active = true;
    count_++;
    return true; // Message scheduled successfully
  } else {
    Serial.println("Advertencia: Límite de mensajes programados alcanzado!");
    return false; // Could not schedule the message
  }
}

void MessageScheduler::processScheduledMessages() {
  unsigned long currentTime = millis();
  for (int i = 0; i < count_; ++i) {
    if (scheduledMessages_[i].active && currentTime >= scheduledMessages_[i].triggerTime) {
      messageQueue_.enqueue(scheduledMessages_[i].message);
      scheduledMessages_[i].active = false; // Mark as processed
      Serial.print("Sacando mensaje!");
      Serial.println(currentTime);

      // Shift remaining active messages to the left to fill the gap and maintain sorted order
      for (int j = i; j < count_ - 1; ++j) {
        scheduledMessages_[j] = scheduledMessages_[j + 1];
      }
      count_--;
      i--; // Re-check the current index as the next message has shifted
    }
  }

  // After processing, ensure any inactive slots at the end are marked as such
  for (int i = count_; i < MAX_SCHEDULED_MESSAGES; ++i) {
    scheduledMessages_[i].active = false;
  }
}
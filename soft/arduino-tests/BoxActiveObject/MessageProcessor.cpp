#include "MessageProcessor.h"
#include "Message.h"
#include "MessagePrintTimed.h" // Include para MessagePrintTimed
#include <Arduino.h>

MessageProcessor::MessageProcessor(MessageQueue& messageQueue)
    : messageQueue_(messageQueue) {}

void MessageProcessor::processMessages() {
  Message* message = messageQueue_.dequeue();
  if (message != nullptr) {
    message->execute();
    if (message->getType() == MessageType::PRINT_TIMED) {
      MessagePrintTimed* timedMessage = static_cast<MessagePrintTimed*>(message);
      if (timedMessage->getStartTime() > 0 && (millis() - timedMessage->getStartTime() >= timedMessage->getDelayMs())) {
        messageQueue_.release(message); // Liberar después de la impresión
      } else if (timedMessage->getStartTime() == 0) {
        // Si es un mensaje con retardo y aún no se ha iniciado el tiempo,
        // lo volvemos a encolar.
        messageQueue_.enqueue(message);
      }
    } else {
      messageQueue_.release(message); // Liberar otros tipos de mensajes inmediatamente
    }
  }
}
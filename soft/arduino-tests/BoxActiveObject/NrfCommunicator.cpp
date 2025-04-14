#include "NrfCommunicator.h"
#include "MessageNrfRx.h"
#include "MessageNrfTx.h"
#include <Arduino.h>
#include <string.h>

NrfCommunicator::NrfCommunicator(RF24 &radio, MessageQueue &messageQueue)
    : radio_(radio), messageQueue_(messageQueue) {
  if (!radio_.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {
    } // hold in infinite loop
  }
  radio_.setPALevel(RF24_PA_LOW);
  radio_.setDataRate(RF24_1MBPS);
  radio_.openReadingPipe(1, NRF_READ_ADDRESS);
  radio_.openWritingPipe(NRF_WRITE_ADDRESS);
  radio_.startListening(); // Inicialmente en modo escucha
}

void NrfCommunicator::processNrf() {
  // Primero, verificar si hay mensajes para enviar
  Message *msgToSend = nullptr;
  for (int i = 0; i < messageQueue_.getCount(); ++i) {
    Message *msg = messageQueue_.peek(i);
    if (msg && msg->getType() == MessageType::NRF_TX) {
      msgToSend = msg;
      break;
    }
  }

  if (msgToSend) {
    radio_.stopListening();
    sendMessage(msgToSend->getPayload());
    messageQueue_.dequeue(msgToSend); // Desencolar después de enviar
    radio_.startListening();
    msgToSend = nullptr;
  }

  // Luego, verificar si hay mensajes recibidos
  receiveMessage();
}

void NrfCommunicator::sendMessage(const char *data) {
  radio_.write(data, strlen(data) + 1);
  Serial.print("NRF Enviado: ");
  Serial.println(data);
}

void NrfCommunicator::receiveMessage() {
  if (radio_.available()) {
    char text[32];
    radio_.read(text, sizeof(text));
    MessageNrfRx *msg =
        messageQueue_.allocate<MessageNrfRx>(static_cast<const char *>(text));
    if (msg != nullptr) {
      messageQueue_.enqueue(msg);
    } else {
      Serial.println("Pool de MessageNrfRx lleno!");
    }
  }
}
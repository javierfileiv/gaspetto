#include "ButtonDetector.h"
#include "Message.h"
#include "MessageButton.h"
#include "MessageNrfRx.h"
#include "MessageNrfTx.h"
#include "MessagePool.h"
#include "MessagePrint.h"
#include "MessagePrintTimed.h"
#include "MessageProcessor.h"
#include "MessageQueue.h"
#include "MessageScheduler.h" // Include MessageScheduler
// #include "NrfCommunicator.h"
#include "NrfConfig.h"
#include <RF24.h>
#include <SPI.h>

const byte BUTTON_PIN = 2;

//set directories /home/fixp/sourceCode/gaspetto/soft/arduino-tests/BoxActiveObject
MessageQueue messageQueue;
ButtonDetector buttonDetector(BUTTON_PIN, messageQueue);
MessageProcessor messageProcessor(messageQueue);
MessageScheduler
    messageScheduler(messageQueue); // Instancia del MessageScheduler


// NRF24L01+ Radio
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
// NrfCommunicator nrfCommunicator(radio, messageQueue);

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando detección de botón con Active Object (Message "
                 "Scheduling con Array Estático)...");

  MessagePrint *startMessage =
      messageQueue.allocate<MessagePrint>("¡Sistema iniciado!");
  if (startMessage != nullptr) {
    messageQueue.enqueue(startMessage);
  } else {
    Serial.println("Pool de MessagePrint lleno al inicio!");
  }
}

void loop() {
  buttonDetector.checkButton();
  messageProcessor.processMessages();
//   nrfCommunicator.processNrf();
  messageScheduler.processScheduledMessages(); // Procesar mensajes programados

  static unsigned long lastScheduleTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastScheduleTime > 5000) {
    MessagePrint *delayedMessage = messageQueue.allocate<MessagePrint>(
        "Mensaje con retardo de 3 segundos (Array Estático)");
    if (delayedMessage != nullptr) {
      if (messageScheduler.scheduleMessage(delayedMessage, 3000)) {
        Serial.print("Mensaje scheduled at ");
        lastScheduleTime = currentTime;
        Serial.println(lastScheduleTime);
      } else {
        Serial.println("Error al programar mensaje: Límite alcanzado.");
        messageQueue.release(delayedMessage); // Liberar si no se pudo programar
      }
    } else {
      Serial.println("Pool de MessagePrint lleno al programar!");
    }

    MessageNrfTx *delayedNrfMessage = messageQueue.allocate<MessageNrfTx>(
        "Mensaje NRF con retardo de 7 segundos (Array Estático)");
    if (delayedNrfMessage != nullptr) {
      if (!messageScheduler.scheduleMessage(delayedNrfMessage, 7000)) {
        Serial.println("Error al programar mensaje NRF: Límite alcanzado.");
        messageQueue.release(
            delayedNrfMessage); // Liberar si no se pudo programar
      } else {
        Serial.println("Mensaje NRF programado.");
      }
    } else {
      Serial.println("Pool de MessageNrfTx lleno al programar NRF!");
    }
  }
}
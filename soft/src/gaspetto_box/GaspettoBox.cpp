#include "GaspettoBox.h"
#include "ActiveObject .h"
#include "State.h"
#include <iostream>

GaspettoBox::GaspettoBox(State *idle, State *running, EventQueue *queue,
                         StateId initial_state)
    : ActiveObject(queue, nullptr) {
  InitMachine(StateId::IDLE, idle);
  InitMachine(StateId::PROCESSING, running);
  SetInitialState(initial_state);
}

void GaspettoBox::debounceAndEnqueue(Event &evt, unsigned long currentTime) {
#ifdef ARDUINO
  if (currentTime - lastDebounceTime > debounceDelay) {
    lastDebounceTime = currentTime;
    if (!eventQueue->full()) {
      eventQueue->enqueue(evt);
      Serial.println("Exiting low-power mode...\n");
      lowPowerMode = false; /*  Wake the system. */
    } else {
      Serial.println("Event queue is full! Unable to enqueue event.\n");
    }
  }
#else
  if (!eventQueue->IsFull()) {
    postEvent(evt);
    Serial.println("Button pressed. Exiting low-power mode...\n");
  } else {
    Serial.println("Event queue is full! Unable to enqueue event.\n");
  }
#endif
}

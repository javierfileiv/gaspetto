#pragma once

#include "ActiveObject .h"
#include "Arduino.h"
#include "EventQueue.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "State.h"
#include <cstdint>
#define __ASSERT_USE_STDERR
#include <assert.h>

class GaspettoCar : public ActiveObject {
public:
  GaspettoCar(State *idle, State *running, EventQueue *queue,
              StateId initial_state);

  void enqueue_random_commands(const uint8_t num_events);
};
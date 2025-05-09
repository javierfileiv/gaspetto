#include "Arduino.h"
#include "EventQueue.h"
#include "IdleState.h"
#include "ProcessingState.h"
#include "State.h"
#include <cstdint>
#define __ASSERT_USE_STDERR
#include <assert.h>
class GaspettoBox {

public:
  GaspettoBox(State *idle, State *running, EventQueue *queue,
              StateId initial_state);

  void transitionTo(StateId newStateId);
  EventQueue getEventQueue() { return *eventQueue; }
  void processNextEvent(void);
  int postEvent(Event evt);
  void enterLowPowerMode(void);
  void Init(void);
  void debounceAndEnqueue(Event &evt,unsigned long currentTime);

private:
  void UpdateState(StateId &newStateId);

private:
  EventQueue *eventQueue;
  State *states[static_cast<int>(StateId::MAX_STATE_ID)];
  StateId currentStateId;
};
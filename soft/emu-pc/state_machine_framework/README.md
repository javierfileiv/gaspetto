# State Machine Framework

An event-driven state machine framework designed for embedded systems, providing a robust foundation for building reactive applications with clear state management and event handling.

## Overview

This framework implements the State pattern combined with an Active Object pattern, enabling:
- **Event-driven architecture**: React to events rather than polling
- **Clear state separation**: Each state encapsulates its own behavior
- **Timed events**: Schedule events for future execution
- **Logging support**: Built-in logging infrastructure
- **Testability**: Easy to mock and test state transitions

## Architecture

### Core Components

```
state_machine_framework/
├── include/
│   ├── ActiveObject.h        # Active object with state machine
│   ├── State.h               # Base state class
│   ├── Event.h               # Event definition
│   ├── EventQueue.h          # Event queue management
│   ├── TimeredEventQueue.h   # Scheduled event queue
│   ├── Log.h                 # Logging interface
│   ├── LogData.h             # Log data structures
│   ├── config_event.h        # Event type definitions
└── src/
    └── TimeredEventQueue.cpp # Timed event implementation
```

## Key Concepts

### Active Object

An **Active Object** is an object with its own event queue and state machine. It processes events sequentially, ensuring thread-safety without explicit locking.

```cpp
class MyActiveObject : public ActiveObject {
public:
    MyActiveObject(EventQueue* queue, TimeredEventQueue* timedQueue)
        : ActiveObject(queue, timedQueue) {}

    void init(StateId initialState) {
        initMachine(StateId::IDLE, &idleState);
        initMachine(StateId::ACTIVE, &activeState);
        ActiveObject::init(initialState);
    }
};
```

### State

Each **State** represents a discrete mode of operation with its own event handlers:

```cpp
class IdleState : public State {
public:
    void entry() override {
        // Called when entering this state
    }

    void exit() override {
        // Called when leaving this state
    }

    void react(const Event& evt) override {
        switch(evt.eventId) {
            case EventId::ACTION:
                // Handle action event
                transitionTo(StateId::ACTIVE);
                break;
        }
    }
};
```

### Event

**Events** are the messages that drive state transitions and actions:

```cpp
struct Event {
    EventId eventId;     // Type of event
    CommandId commandId; // Optional command data
    uint32_t param1;     // Optional parameters
    uint32_t param2;
};
```

Common event types (defined in `config_event.h`):
- `EventId::ACTION` - User action/command
- `EventId::TIMEOUT` - Timer expired
- `EventId::RADIO_EVENT` - Radio communication event

### Event Queue

The **EventQueue** manages incoming events:

```cpp
EventQueue queue;

// Send an event
Event evt{EventId::ACTION, CommandId::START};
queue.sendEvent(&evt);

// Retrieve next event
Event* nextEvt = queue.getEvent();
if (nextEvt) {
    currentState->react(*nextEvt);
}
```

### Timered Event Queue

The **TimeredEventQueue** enables scheduling events for future execution:

```cpp
TimeredEventQueue timedQueue;

// Schedule event for 1000ms from now
Event evt{EventId::TIMEOUT, CommandId::STOP};
timedQueue.sendEvent(&evt, 1000);

// Check for expired timers
timedQueue.update();  // Call periodically (e.g., in main loop)
```

## Event Flow

```
[External Input]
       ↓
[EventQueue.sendEvent()]
       ↓
[ActiveObject.processEvents()]
       ↓
[CurrentState.react()]
       ↓
[State Actions / Transitions]
```

## State Lifecycle

```
[transitionTo(NewState)]
       ↓
[CurrentState.exit()]
       ↓
[CurrentState = NewState]
       ↓
[NewState.entry()]
```

## Usage Example

### Define States

```cpp
class IdleState : public State {
public:
    void entry() override {
        Serial.println("Entering Idle State");
        // Set low power mode
    }

    void react(const Event& evt) override {
        if (evt.eventId == EventId::ACTION) {
            if (evt.commandId == CommandId::START) {
                transitionTo(StateId::PROCESSING);
            }
        }
    }
};

class ProcessingState : public State {
public:
    void entry() override {
        Serial.println("Entering Processing State");
        // Schedule timeout
        Event timeout{EventId::TIMEOUT};
        machine->getTimeredQueue()->sendEvent(&timeout, 5000);
    }

    void react(const Event& evt) override {
        if (evt.eventId == EventId::TIMEOUT) {
            transitionTo(StateId::IDLE);
        }
    }
};
```

### Create Active Object

```cpp
class MySystem : public ActiveObject {
public:
    MySystem(EventQueue* queue, TimeredEventQueue* timedQueue)
        : ActiveObject(queue, timedQueue)
        , idleState()
        , processingState() {}

    void init() {
        initMachine(StateId::IDLE, &idleState);
        initMachine(StateId::PROCESSING, &processingState);
        ActiveObject::init(StateId::IDLE);
    }

private:
    IdleState idleState;
    ProcessingState processingState;
};
```

### Main Loop

```cpp
EventQueue eventQueue;
TimeredEventQueue timedQueue;
MySystem system(&eventQueue, &timedQueue);

void setup() {
    system.init();
}

void loop() {
    // Update timers
    timedQueue.update();

    // Process events
    Event* evt = eventQueue.getEvent();
    if (evt) {
        system.getCurrentState()->react(*evt);
    }

    // External event injection
    if (buttonPressed()) {
        Event evt{EventId::ACTION, CommandId::START};
        eventQueue.sendEvent(&evt);
    }
}
```

## Logging

The framework includes a logging system:

```cpp
class MyState : public State {
    void react(const Event& evt) override {
        logInfo("Processing event");

        if (error) {
            logError("Error occurred", errorCode);
        }
    }
};
```

Log levels:
- `logDebug()` - Detailed debugging information
- `logInfo()` - General information
- `logWarning()` - Warning conditions
- `logError()` - Error conditions

## Testing

The framework is designed for testability:

```cpp
TEST(StateMachineTest, TransitionsCorrectly) {
    EventQueue queue;
    TimeredEventQueue timedQueue;
    MySystem system(&queue, &timedQueue);

    system.init();
    ASSERT_EQ(system.getCurrentState()->getId(), StateId::IDLE);

    Event evt{EventId::ACTION, CommandId::START};
    queue.sendEvent(&evt);

    Event* received = queue.getEvent();
    system.getCurrentState()->react(*received);

    ASSERT_EQ(system.getCurrentState()->getId(), StateId::PROCESSING);
}
```

## Design Patterns

This framework combines several design patterns:

- **State Pattern**: Encapsulate state-specific behavior
- **Active Object**: Concurrent execution with event queues
- **Command Pattern**: Events as commands
- **Template Method**: State lifecycle hooks (entry/exit/react)

## Benefits

### Maintainability
- Clear separation of state-specific logic
- Easy to add new states without modifying existing ones
- Centralized event handling

### Testability
- States can be tested in isolation
- Event sequences can be scripted and verified
- No direct hardware dependencies

### Flexibility
- Easy to add new event types
- States can be reused across different systems
- Supports hierarchical state machines (can be extended)

### Reliability
- Sequential event processing eliminates race conditions
- Clear state transitions prevent undefined behavior
- Timed events ensure timeout handling

## Configuration

Event types are defined in `config_event.h`:

```cpp
enum class EventId {
    NONE = 0,
    ACTION,
    TIMEOUT,
    RADIO_EVENT,
    // Add custom event types
};

enum class StateId {
    IDLE = 0,
    PROCESSING,
    // Add custom states
};
```

## Advanced Features

### Context Passing

Share resources across states using a context structure:

```cpp
struct Context {
    EventQueue* eventQueue;
    MovementController* movementController;
    RadioController* radioController;
};

class MyState : public State {
    void react(const Event& evt) override {
        Context* ctx = static_cast<Context*>(machine->getContext());
        ctx->movementController->stop();
    }
};
```

### Hierarchical States

Extend the framework to support substates:

```cpp
class SuperState : public State {
protected:
    State* substate;

public:
    void react(const Event& evt) override {
        // Try substate first
        if (substate && substate->handle(evt)) {
            return;
        }
        // Handle at this level
        handleLocally(evt);
    }
};
```

## Best Practices

1. **Keep states focused**: Each state should have a single responsibility
2. **Use entry/exit**: Initialize/cleanup in entry/exit methods
3. **Avoid long processing**: React quickly, defer heavy work
4. **Use timed events**: Prefer scheduled events over polling
5. **Log transitions**: Log state changes for debugging
6. **Test exhaustively**: Test all possible event/state combinations

## Performance

- Event queue operations: O(1)
- State lookup: O(1) array indexing
- Timer management: O(n) where n is number of pending timers
- Memory: Minimal overhead, no dynamic allocation

Suitable for resource-constrained embedded systems.

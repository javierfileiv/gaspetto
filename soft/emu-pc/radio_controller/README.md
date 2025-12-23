# Radio Controller

Wireless communication module for bidirectional data exchange using nRF24L01+ radio transceivers. Provides event-driven radio communication with automatic retransmission and acknowledgment.

## Overview

The Radio Controller manages RF24 (nRF24L01+) wireless communication, integrating it with the event-driven state machine framework. It handles:
- Radio initialization and configuration
- Event transmission and reception
- Auto-acknowledgment with payload
- Integration with event queues

## Hardware

**nRF24L01+** is a 2.4GHz radio transceiver featuring:
- Up to 2Mbps data rate
- 125 frequency channels
- Auto-acknowledgment and retransmission
- 6 data pipes for multi-device communication
- Low power consumption

## Structure

```
radio_controller/
├── include/
│   ├── RadioController.h    # Main radio controller class
│   └── config_radio.h        # Radio configuration constants
└── src/
    └── RadioController.cpp   # Implementation
```

## Configuration

### Pipes (config_radio.h)

Communication uses named pipes for addressing:

```cpp
// Pipe addresses (must be 5 bytes)
const uint8_t gaspetto_car_pipe_name[] = "Car01";
const uint8_t gaspetto_box_pipe_name[] = "Box01";
```

### Radio Parameters

```cpp
// Frequency channel (0-125)
#define RADIO_CHANNEL 76

// Power level
#define RADIO_PA_LEVEL RF24_PA_MAX  // Maximum power

// Data rate
#define RADIO_DATA_RATE RF24_1MBPS  // 1Mbps
```

## API Reference

### Initialization

```cpp
RadioController(RF24& radio, EventQueue* queue,
                const uint8_t* writingPipe,
                const uint8_t* readingPipe);
```

**Parameters:**
- `radio` - RF24 transceiver instance
- `queue` - Event queue for received events
- `writingPipe` - Address for transmitted data
- `readingPipe` - Address for received data

**Example:**
```cpp
RF24 radio(CE_PIN, CSN_PIN);
EventQueue eventQueue;

RadioController controller(radio, &eventQueue,
                          gaspetto_box_pipe_name,  // We transmit to Box
                          gaspetto_car_pipe_name); // We receive as Car

controller.init();
```

### Basic Operations

#### init()
```cpp
bool init();
```

Initialize and configure the radio:
- Sets channel, power level, data rate
- Opens reading and writing pipes
- Starts listening mode

**Returns:** `true` on success

#### processRadioEvent()
```cpp
void processRadioEvent();
```

Check for received data and post to event queue:
- Polls radio for available data
- Reads received event
- Posts to event queue
- Should be called regularly in main loop

#### sendEvent()
```cpp
bool sendEvent(Event* evt);
```

Transmit an event:
- Stops listening temporarily
- Writes event data
- Resumes listening
- Uses auto-acknowledgment

**Returns:** `true` if transmission succeeded

## Usage Patterns

### Car Side (Receiver/Responder)

```cpp
// Car receives commands from box, sends responses
RF24 radio(CE_PIN, CSN_PIN);
EventQueue eventQueue;

RadioController radioController(radio, &eventQueue,
                               gaspetto_box_pipe_name,  // Transmit to box
                               gaspetto_car_pipe_name); // Receive as car

void setup() {
    radioController.init();
}

void loop() {
    // Check for received commands
    radioController.processRadioEvent();

    // Process events from queue
    Event* evt = eventQueue.getEvent();
    if (evt) {
        handleCommand(evt);

        // Send response
        Event response{EventId::STATUS, StatusId::OK};
        radioController.sendEvent(&response);
    }
}
```

### Box Side (Controller/Sender)

```cpp
// Box sends commands to car, receives responses
RF24 radio(CE_PIN, CSN_PIN);
EventQueue eventQueue;

RadioController radioController(radio, &eventQueue,
                               gaspetto_car_pipe_name,  // Transmit to car
                               gaspetto_box_pipe_name); // Receive as box

void loop() {
    if (buttonPressed()) {
        // Send command
        Event cmd{EventId::ACTION, CommandId::MOTOR_FORWARD};
        if (radioController.sendEvent(&cmd)) {
            Serial.println("Command sent");
        }
    }

    // Check for responses
    radioController.processRadioEvent();
}
```

## Event Structure

Events transmitted over radio must fit within RF24 payload size (32 bytes):

```cpp
struct Event {
    EventId eventId;      // 4 bytes
    CommandId commandId;  // 4 bytes
    uint32_t param1;      // 4 bytes
    uint32_t param2;      // 4 bytes
};  // Total: 16 bytes
```

## Communication Protocol

### Command Flow

```
Box (Controller)              Car (Receiver)
     |                             |
     |--- Event (MOTOR_FORWARD) -->|
     |                             | Process command
     |                             | Execute action
     |<--- Event (STATUS_OK) ------|
     |                             |
```

### Auto-Acknowledgment

The nRF24L01+ provides hardware-level acknowledgment:
1. Transmitter sends packet
2. Receiver auto-sends ACK
3. If no ACK received, auto-retry (up to 15 times)
4. `sendEvent()` returns success/failure

### Payload with ACK

The radio supports sending response data with the ACK:
- Receiver can attach payload to ACK packet
- Enables immediate bidirectional communication
- Reduces latency for request-response patterns

## Error Handling

### Transmission Failures

```cpp
if (!radioController.sendEvent(&evt)) {
    // Transmission failed after retries
    Serial.println("Radio communication error");
    // Retry or enter error state
}
```

### Reception Handling

```cpp
void processRadioEvent() {
    if (radio.available()) {
        Event evt;
        radio.read(&evt, sizeof(Event));

        // Validate event
        if (isValidEvent(evt)) {
            eventQueue->sendEvent(&evt);
        }
    }
}
```

## Testing

### With Real Hardware

```cpp
// Test transmission
Event testEvent{EventId::ACTION, CommandId::TEST};
bool success = radioController.sendEvent(&testEvent);
ASSERT_TRUE(success);
```

### With Mocks (PC Testing)

```cpp
// tests/mocks/mock_RF24.h
class MockRF24 : public RF24 {
public:
    MOCK_METHOD(bool, begin, (), (override));
    MOCK_METHOD(bool, write, (const void*, uint8_t), (override));
    MOCK_METHOD(bool, available, (), (override));
    // ...
};

TEST(RadioControllerTest, SendsEvent) {
    MockRF24 mockRadio;
    EventQueue queue;
    RadioController controller(mockRadio, &queue, pipe1, pipe2);

    EXPECT_CALL(mockRadio, write(_, _)).WillOnce(Return(true));

    Event evt{EventId::ACTION};
    ASSERT_TRUE(controller.sendEvent(&evt));
}
```

## Performance Considerations

### Polling vs Interrupts

Current implementation uses polling:
```cpp
void loop() {
    radioController.processRadioEvent();  // Check regularly
    // Other tasks
}
```

For better performance, use IRQ pin:
```cpp
volatile bool radioInterrupt = false;

void radioISR() {
    radioInterrupt = true;
}

void setup() {
    attachInterrupt(IRQ_PIN, radioISR, FALLING);
    radioController.init();
}

void loop() {
    if (radioInterrupt) {
        radioInterrupt = false;
        radioController.processRadioEvent();
    }
}
```

### Transmission Timing

- Single event transmission: ~1-5ms (depending on retries)
- Maximum throughput: ~200-500 events/second
- Auto-retry delay: Configurable (default 250µs)

## Advanced Configuration

### Multi-Pipe Communication

Support multiple devices:

```cpp
// Open additional reading pipes
radio.openReadingPipe(1, pipe1);
radio.openReadingPipe(2, pipe2);
radio.openReadingPipe(3, pipe3);

// Check which pipe received data
uint8_t pipe;
if (radio.available(&pipe)) {
    // Handle based on pipe number
}
```

### Dynamic Payload Size

Enable variable payload sizes:

```cpp
radio.enableDynamicPayloads();
radio.enableAckPayload();
```

### Power Management

Adjust power level based on distance:

```cpp
radio.setPALevel(RF24_PA_LOW);   // Short range, low power
radio.setPALevel(RF24_PA_HIGH);  // Medium range
radio.setPALevel(RF24_PA_MAX);   // Maximum range
```

## Troubleshooting

### No Communication

1. **Check wiring**: Verify SPI connections (MISO, MOSI, SCK, CE, CSN)
2. **Check power**: nRF24L01+ requires stable 3.3V
3. **Check channel**: Ensure both devices use same channel
4. **Check pipes**: Verify pipe addresses match

### Intermittent Communication

1. **Add capacitor**: 10µF across VCC/GND near radio
2. **Reduce distance**: Start with devices close together
3. **Reduce power**: Sometimes lower power is more stable
4. **Change channel**: Avoid crowded 2.4GHz channels (WiFi)

### Debugging

```cpp
void printRadioDetails() {
    radio.printDetails();  // Outputs full configuration to Serial
}
```

## Safety Considerations

- **Timeout handling**: Don't wait indefinitely for response
- **Command validation**: Verify received events are valid
- **Fail-safe**: Enter safe state on communication loss
- **Watchdog**: Implement timeout for critical commands

## References

- [nRF24L01+ Datasheet](https://www.nordicsemi.com/products/nrf24l01)
- [RF24 Library Documentation](https://nrf24.github.io/RF24/)
- [nRF24L01+ Tutorial](https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/)

# Gaspetto Box Target

Firmware for the remote control box that sends commands to the Gaspetto Car via RF24 wireless communication.

## Overview

The Gaspetto Box is the controller/transmitter that:
- Reads user input (buttons, joystick, etc.)
- Sends movement commands to the car via nRF24L01+ radio
- Receives status/telemetry from the car
- Provides user feedback (LEDs, display, etc.)

## Structure

```
gaspetto_box/
├── CMakeLists.txt
├── send_values.cpp      # Simple value transmission demo
├── include/             # Header files (to be added)
└── src/                 # Source files (to be added)
```

## Current Status

The box target currently contains a simple demonstration program (`send_values.cpp`) that shows basic radio transmission. Full controller implementation is planned.

## Planned Features

### Input Handling
- **Joystick**: Analog control for speed and direction
- **Buttons**: Discrete commands (stop, mode select, etc.)
- **Switches**: Feature enable/disable
- **Potentiometers**: Speed/sensitivity adjustment

### Command Types

#### Basic Movement
- Forward/Backward
- Left/Right turn
- Speed control
- Emergency stop

#### Advanced Control
- PID straight driving mode
- Precision turning mode
- Speed profiles (slow/medium/fast)
- Autonomous modes

### User Feedback
- **LEDs**: Connection status, mode indicators
- **Display**: Speed, battery, connection strength
- **Buzzer**: Audio feedback for actions

### Communication
- Bidirectional radio link with car
- Command acknowledgment
- Telemetry reception
- Connection monitoring

## Building

### For PC Emulation
```bash
cd build-box
cmake -DGASPETTO_BOX=ON -DCMAKE_BUILD_TYPE=Release ..
make gaspetto_box
```

### For Embedded Target
```bash
cmake -DGASPETTO_BOX=ON -DCMAKE_TOOLCHAIN_FILE=stm32_toolchain.cmake ..
make gaspetto_box
st-flash write gaspetto_box.bin 0x8000000
```

## Architecture (Planned)

### State Machine
Similar to car, but with controller-specific states:
- **Idle**: Waiting for user input
- **Active**: Sending commands
- **Telemetry**: Displaying car status
- **Configuration**: Settings adjustment

### Input Processing
```cpp
void processInputs() {
    // Read joystick
    int x = analogRead(JOYSTICK_X);
    int y = analogRead(JOYSTICK_Y);

    // Convert to command
    Event cmd = joystickToCommand(x, y);

    // Send to car
    radioController.sendEvent(&cmd);
}
```

### Telemetry Display
```cpp
void displayTelemetry() {
    // Receive status from car
    if (radioController.hasData()) {
        Status status;
        radioController.receive(&status);

        // Update display
        display.print("Speed: ");
        display.println(status.speed);
        display.print("Battery: ");
        display.println(status.battery);
    }
}
```

## Pin Configuration (Planned)

### Radio Module
- CE: Digital pin
- CSN: SPI CS pin
- MOSI: SPI MOSI
- MISO: SPI MISO
- SCK: SPI SCK

### Input Devices
- Joystick X: Analog pin A0
- Joystick Y: Analog pin A1
- Button 1: Digital pin (pull-up)
- Button 2: Digital pin (pull-up)
- Emergency Stop: Digital pin (hardware interrupt)

### Output Devices
- Status LED: Digital pin (PWM)
- Connection LED: Digital pin
- Buzzer: Digital pin (PWM)
- Display: I2C (SDA/SCL)

## Command Protocol

### Basic Commands
```cpp
// Stop
Event stop{EventId::ACTION, CommandId::MOTOR_STOP, 0, 0};

// Forward at 70% speed
Event forward{EventId::ACTION, CommandId::MOTOR_FORWARD, 70, 0};

// Turn right at 60% speed
Event turnRight{EventId::ACTION, CommandId::MOTOR_RIGHT, 60, 0};
```

### Joystick Mapping
```cpp
Event joystickToCommand(int x, int y) {
    // Center: (512, 512) on 10-bit ADC
    int dx = x - 512;
    int dy = y - 512;

    // Dead zone
    if (abs(dx) < 50 && abs(dy) < 50) {
        return Event{EventId::ACTION, CommandId::MOTOR_STOP};
    }

    // Forward/Backward
    if (abs(dy) > abs(dx)) {
        if (dy > 0) {
            return Event{EventId::ACTION, CommandId::MOTOR_FORWARD, mapSpeed(dy)};
        } else {
            return Event{EventId::ACTION, CommandId::MOTOR_BACKWARD, mapSpeed(-dy)};
        }
    }

    // Left/Right
    if (dx > 0) {
        return Event{EventId::ACTION, CommandId::MOTOR_RIGHT, mapSpeed(dx)};
    } else {
        return Event{EventId::ACTION, CommandId::MOTOR_LEFT, mapSpeed(-dx)};
    }
}

uint8_t mapSpeed(int value) {
    // Map 50-512 to 30-100%
    return map(value, 50, 512, 30, 100);
}
```

## Safety Features

### Watchdog
- Reset if no user input for extended period
- Prevents runaway transmission

### Connection Monitoring
```cpp
void checkConnection() {
    if (millis() - lastAckTime > CONNECTION_TIMEOUT) {
        // Lost connection
        connectionLost = true;
        setLED(RED, BLINK);

        // Send stop command
        sendStopCommand();
    }
}
```

### Emergency Stop
```cpp
void EMERGENCY_STOP_ISR() {
    // Hardware interrupt for immediate stop
    Event stop{EventId::ACTION, CommandId::MOTOR_STOP};
    radioController.sendEvent(&stop);

    // Disable further commands
    emergencyMode = true;
}
```

## Power Management

- **Active Mode**: Transmitting commands, display on
- **Idle Mode**: No input, display dimmed
- **Sleep Mode**: Extended inactivity, deep sleep

## User Interface (Planned)

### LCD Display Layout
```
+------------------+
| Gaspetto Control |
| Speed: 75%       |
| Batt:  85%       |
| [||||||||||||  ] |
+------------------+
```

### LED Indicators
- **Green**: Connected and ready
- **Yellow**: Transmitting command
- **Red**: Connection lost
- **Blue**: Receiving telemetry

## Testing

### Hardware Testing
1. **Radio Test**: Verify communication with car
2. **Input Test**: Check all buttons and joystick
3. **Display Test**: Verify output devices
4. **Range Test**: Maximum communication distance

### Unit Testing
```cpp
TEST(BoxControllerTest, JoystickForward) {
    Event cmd = joystickToCommand(512, 1000);
    ASSERT_EQ(cmd.commandId, CommandId::MOTOR_FORWARD);
}

TEST(BoxControllerTest, EmergencyStop) {
    triggerEmergencyStop();
    ASSERT_TRUE(emergencyMode);
    // Verify stop command sent
}
```

## Configuration

### Radio Settings
Match car configuration:
```cpp
#define RADIO_CHANNEL 76
#define RADIO_PA_LEVEL RF24_PA_MAX
#define RADIO_DATA_RATE RF24_1MBPS
```

### Timing
```cpp
#define COMMAND_INTERVAL_MS 50      // Send commands every 50ms
#define TELEMETRY_INTERVAL_MS 200   // Request telemetry every 200ms
#define CONNECTION_TIMEOUT_MS 1000  // Consider lost after 1s
```

## Development Roadmap

- [ ] Implement state machine architecture
- [ ] Add joystick input handling
- [ ] Create button debouncing
- [ ] Add LCD/OLED display support
- [ ] Implement telemetry reception
- [ ] Add battery monitoring
- [ ] Create configuration menu
- [ ] Add data logging
- [ ] Implement firmware update capability

## References

- See [Gaspetto Car README](../gaspetto_car/README.md) for command protocol
- See [Radio Controller README](../../radio_controller/README.md) for communication details

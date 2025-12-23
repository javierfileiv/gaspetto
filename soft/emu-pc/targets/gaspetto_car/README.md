# Gaspetto Car Target

Firmware for the remote-controlled car, featuring event-driven state machine, radio communication, PID-based movement control with IMU integration, and motor control.

## Overview

The Gaspetto Car is an autonomous/remote-controlled vehicle that:
- Receives commands wirelessly via nRF24L01+ radio
- Controls two motors independently (differential drive)
- Uses IMU (MPU6050) for orientation tracking
- Implements PID control for straight driving and precise turning
- Operates in event-driven low-power states

## Architecture

```
gaspetto_car/
├── CMakeLists.txt
├── include/
│   ├── GaspettoCar.h           # Main car state machine
│   ├── Context.h               # Shared context structure
│   ├── IdleState.h             # Low-power waiting state
│   └── ProcessingState.h       # Active command processing
├── movement_controller/
│   ├── include/
│   │   ├── MovementController.h    # High-level movement API
│   │   ├── MotorControl.h          # Motor driver interface
│   │   ├── MotorControlInterface.h # Abstract motor interface
│   │   ├── IMUOrientation.h        # IMU sensor interface
│   │   └── IMUOrientationInterface.h # Abstract IMU interface
│   └── src/
│       ├── MovementController.cpp  # Movement implementation
│       ├── MotorControl.cpp        # Motor control logic
│       └── IMUOrientation.cpp      # IMU sensor handling
└── src/
    ├── main.cpp                # Application entry point
    ├── GaspettoCar.cpp         # State machine implementation
    ├── IdleState.cpp           # Idle state behavior
    └── ProcessingState.cpp     # Processing state behavior
```

## Features

### State Machine

#### Idle State
- **Purpose**: Low-power waiting mode
- **Behavior**:
  - Listens for radio commands
  - Enters low-power mode between events
  - Transitions to Processing on action events

#### Processing State
- **Purpose**: Active command execution
- **Behavior**:
  - Executes motor commands
  - Updates movement control (PID loops)
  - Returns to Idle when command completes

### Movement Control

The car supports multiple movement modes:

#### Direct Motor Control
Set left and right motor speeds independently:
```cpp
movementController.setMotorSpeeds(
    true, 80,    // Left: forward, 80% speed
    true, 75     // Right: forward, 75% speed
);
```

#### PID-based Straight Driving
Maintains straight line using IMU feedback:
```cpp
movementController.startStraightDriving(
    true,   // forward
    70,     // 70% speed
    180     // target: 180° (forward)
);
```

**How it works:**
1. IMU measures current heading (yaw angle)
2. PID controller calculates correction
3. Differential speed applied to motors
4. Car maintains straight line despite terrain variations

#### PID-based Turning
Precise in-place rotation:
```cpp
movementController.startTurningInPlace(
    false,  // counter-clockwise
    50      // 50% speed
);
```

**How it works:**
1. Records initial heading from IMU
2. Target heading calculated (±90°)
3. PID controller maintains turn rate
4. Stops when target heading reached

#### Stop
Immediate motor shutdown:
```cpp
movementController.stop();
```

### IMU Integration

**MPU6050** 6-axis IMU provides:
- 3-axis accelerometer
- 3-axis gyroscope
- Orientation tracking (yaw angle)
- Real-time feedback for PID control

Configuration:
- Accelerometer range: ±4g
- Gyroscope range: ±500°/s
- Low-pass filter: 21Hz bandwidth
- Update rate: ~100Hz

### Motor Control

**Dual H-Bridge** motor driver (L298N or similar):
- Independent control of two DC motors
- PWM speed control (0-100%)
- Direction control (forward/backward)
- Hardware PWM on STM32 timer
- Frequency: 20kHz (configurable via MOTOR_FREQ)

Pin configuration:
```cpp
// Left motor
#define MOTOR_LEFT_FWD_PIN  PA0
#define MOTOR_LEFT_BWD_PIN  PA1
#define MOTOR_LEFT_PWM_PIN  PA2

// Right motor
#define MOTOR_RIGHT_FWD_PIN PA3
#define MOTOR_RIGHT_BWD_PIN PA4
#define MOTOR_RIGHT_PWM_PIN PA5
```

## Radio Commands

### Supported Commands

The car responds to these `CommandId` values:

- **MOTOR_FORWARD**: Move forward
- **MOTOR_BACKWARD**: Move backward
- **MOTOR_LEFT**: Turn left
- **MOTOR_RIGHT**: Turn right
- **MOTOR_STOP**: Stop all motors
- **PID_STRAIGHT**: Engage PID straight driving
- **PID_TURN_LEFT**: Turn left 90° (PID)
- **PID_TURN_RIGHT**: Turn right 90° (PID)

### Event Structure

```cpp
Event {
    EventId eventId;       // EVENT_ACTION for commands
    CommandId commandId;   // Motor command
    uint32_t param1;       // Speed (0-100) or distance
    uint32_t param2;       // Additional parameter
}
```

### Command Examples

#### Basic Movement
```cpp
// Forward at 80% speed
Event evt{EventId::ACTION, CommandId::MOTOR_FORWARD, 80, 0};

// Turn right at 60% speed
Event evt{EventId::ACTION, CommandId::MOTOR_RIGHT, 60, 0};

// Stop
Event evt{EventId::ACTION, CommandId::MOTOR_STOP, 0, 0};
```

#### PID-Controlled Movement
```cpp
// Drive straight at 70% speed
Event evt{EventId::ACTION, CommandId::PID_STRAIGHT, 70, 0};

// Turn left 90° at 50% speed
Event evt{EventId::ACTION, CommandId::PID_TURN_LEFT, 50, 0};
```

## Context Structure

Shared resources across states:

```cpp
struct Context {
    EventQueue* mainEventQueue;
    MovementController* movementController;
    RadioController* radioController;
    TimeredEventQueue* timeredEventQueue;
    State* idleState;
    State* processingState;
    uint32_t pwm_freq;
};
```

## Main Loop

```cpp
void loop() {
    // Update timers (for timed events)
    timeredEventQueue.update();

    // Update movement control (PID loops)
    movementController.updateMovement();

    // Check for radio commands
    radioController.processRadioEvent();

    // Process event queue
    Event* evt = eventQueue.getEvent();
    if (evt) {
        car.getCurrentState()->react(*evt);
    }

    // Enter low-power mode if idle
    if (car.getCurrentState() == idleState) {
        enterLowPowerMode();
    }
}
```

## Building

### For PC Emulation
```bash
cd build-car
cmake -DGASPETTO_CAR=ON -DCMAKE_BUILD_TYPE=Release ..
make gaspetto_car
./targets/gaspetto_car/gaspetto_car
```

### For Embedded Target
```bash
# Configure for STM32
cmake -DGASPETTO_CAR=ON -DCMAKE_TOOLCHAIN_FILE=stm32_toolchain.cmake ..
make gaspetto_car

# Flash to device
st-flash write gaspetto_car.bin 0x8000000
```

## Configuration

### PID Tuning

Adjust PID constants in MovementController.cpp:

```cpp
// Straight driving PID
float kp_straight = 2.0;   // Proportional gain
float ki_straight = 0.1;   // Integral gain
float kd_straight = 0.5;   // Derivative gain

// Turning PID
float kp_turn = 3.0;
float ki_turn = 0.2;
float kd_turn = 1.0;
```

### Motor Characteristics

Adjust for your specific motors:

```cpp
// Minimum speed to overcome static friction
#define MIN_SPEED_PERCENT 30

// Maximum speed limit
#define MAX_SPEED_PERCENT 100

// Speed differential for turns
#define TURN_SPEED_DIFF 20
```

### IMU Calibration

Calibrate IMU offsets:

```cpp
void calibrateIMU() {
    // Measure bias over 100 samples
    float gyro_bias_z = 0;
    for (int i = 0; i < 100; i++) {
        sensors_event_t gyro;
        imu.getEvent(nullptr, &gyro, nullptr);
        gyro_bias_z += gyro.gyro.z;
        delay(10);
    }
    gyro_bias_z /= 100.0;

    // Apply correction in IMU reading
}
```

## Testing

### Unit Tests

Test state transitions and command handling:

```cpp
TEST_F(GaspettoCarTest, ForwardCommand) {
    Event cmd{EventId::ACTION, CommandId::MOTOR_FORWARD, 80, 0};

    EXPECT_CALL(mockMotor, setMotorLeft(true, 80));
    EXPECT_CALL(mockMotor, setMotorRight(true, 80));

    eventQueue.sendEvent(&cmd);
    car.processEvents();

    ASSERT_EQ(car.getCurrentState()->getId(), StateId::PROCESSING);
}
```

### Hardware Testing

1. **Motor Test**: Verify direction and speed
2. **IMU Test**: Check orientation readings
3. **Radio Test**: Confirm command reception
4. **PID Test**: Validate straight driving and turning
5. **Integration Test**: Full command sequence

## Troubleshooting

### Car Doesn't Move
- Check motor connections
- Verify power supply (motors need adequate current)
- Test motors directly (bypass control)
- Check PWM signals with oscilloscope

### Car Drifts During Straight Driving
- Calibrate IMU gyroscope bias
- Adjust PID constants (increase Kp)
- Check motor speeds are balanced
- Verify IMU mounting is level

### Erratic Turning
- Reduce turn speed
- Increase PID derivative gain (Kd)
- Check IMU update rate
- Verify yaw calculation

### Radio Not Responding
- Check radio module connections
- Verify pipe addresses match control box
- Test radio with simple ping/pong
- Check interference from motors (add capacitors)

## Safety Features

- **Watchdog Timer**: Resets if main loop hangs
- **Command Timeout**: Stop motors if no command for 1 second
- **Invalid Command**: Ignore malformed radio packets
- **Emergency Stop**: Dedicated stop command priority

## Power Management

- **Active Mode**: Full speed, 100-500mA
- **Idle Mode**: Motors off, radio listening, 50-100mA
- **Low Power Mode**: CPU sleep between events, 20-50mA

## Performance

- **Radio latency**: 5-20ms command to execution
- **PID update rate**: 100Hz
- **Maximum speed**: 1-2 m/s (depends on motors)
- **Turn accuracy**: ±2° with PID control
- **Battery life**: 30-60 minutes (depends on usage)

## Future Enhancements

- [ ] Obstacle avoidance with ultrasonic sensors
- [ ] Line following with IR sensors
- [ ] Odometry for distance tracking
- [ ] Autonomous navigation modes
- [ ] Battery voltage monitoring
- [ ] LED status indicators
- [ ] Multiple speed profiles

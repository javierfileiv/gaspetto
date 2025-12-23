# Arduino Framework - PC Emulation Layer

This directory contains PC-compatible implementations of Arduino APIs and third-party libraries, enabling embedded firmware to be built, tested, and debugged on a standard PC.

## Purpose

The Arduino Framework abstraction layer provides:
- **Cross-platform development**: Write once, run on PC and embedded targets
- **Rapid prototyping**: Fast compilation and immediate execution
- **Hardware-independent testing**: Unit tests without physical hardware
- **Standard debugging tools**: Use GDB, Valgrind, and other PC tools

## Structure

```
arduino_framework/
├── include/              # Arduino API headers
│   ├── Arduino.h         # Core Arduino functions
│   ├── Serial.h          # Serial communication
│   ├── Wire.h            # I2C communication
│   └── __assert.h        # Assertion utilities
├── HardwareTimer/        # PWM timer emulation
├── RF24/                 # nRF24L01+ radio library stub
├── Adafruit_MPU6050/     # IMU sensor library stub
└── implementations/      # Target-specific implementations
```

## Components

### Core Arduino API (`include/`)

#### Arduino.h
Core timing and utility functions:
- `millis()` - Milliseconds since program start (using `std::chrono`)
- `micros()` - Microseconds since program start
- `delay()` - Blocking delay
- `delayMicroseconds()` - Microsecond delay
- Pin manipulation stubs
- Low power mode callback

#### Serial.h / Serial.cpp
Serial communication stub for PC:
- `Serial.begin()` - Initialize (no-op on PC)
- `Serial.print()` / `Serial.println()` - Maps to `std::cout`
- `Serial.available()` - Always returns 0
- `Serial.read()` - Returns 0

#### Wire.h / Wire.cpp
I2C communication stub:
- `Wire.begin()` - Initialize I2C (stub)
- `Wire.beginTransmission()` - Start I2C transaction
- `Wire.write()` - Write data
- `Wire.endTransmission()` - End transaction (always succeeds)
- `Wire.requestFrom()` - Request data from slave
- `Wire.read()` - Read received data (returns 0)

### Hardware Timer (`HardwareTimer/`)

PWM timer emulation for motor control:
- `HardwareTimer` class mimics STM32 hardware timer
- `setPWM()` - Configure PWM frequency and duty cycle
- Stores configuration without actual PWM generation
- Enables testing of motor control logic

### RF24 Radio (`RF24/`)

Stub implementation of RF24 library for nRF24L01+ radio:
- `begin()` - Initialize radio (always succeeds)
- `openWritingPipe()` / `openReadingPipe()` - Configure pipes
- `startListening()` / `stopListening()` - Control RX/TX mode
- `available()` - Check for received data (returns false)
- `read()` / `write()` - Data transfer (stubbed)
- `setChannel()`, `setPALevel()`, `setDataRate()` - Configuration

### Adafruit MPU6050 (`Adafruit_MPU6050/`)

IMU sensor stub with simulated data:
- `begin()` - Initialize sensor (always succeeds)
- `setAccelerometerRange()` - Configure accelerometer
- `setGyroRange()` - Configure gyroscope
- `setFilterBandwidth()` - Configure low-pass filter
- `getEvent()` - Returns simulated sensor data:
  - Accelerometer: (0.1, 0.1, 9.8 m/s²) - Simulates level orientation
  - Gyroscope: (0.01, 0.01, 0.01 rad/s) - Small drift
  - Temperature: 25°C

### Implementations (`implementations/`)

Target-specific main loops and setup:
- `gaspetto_car.cpp` - Car firmware entry point
- `gaspetto_box.cpp` - Control box firmware entry point
- `nrf_sender.cpp` - Radio test utility

## How It Works

### Timing Implementation

Uses `std::chrono` for accurate cross-platform timing:

```cpp
unsigned long millis() {
    static auto start_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
    return static_cast<unsigned long>(duration.count());
}
```

### Serial Output

Maps Arduino Serial to standard output:

```cpp
void Serial::println(const char* msg) {
    std::cout << msg << std::endl;
}
```

### Hardware Stubs

Hardware peripherals return safe default values:
- Radio operations return success/no-data
- I2C operations succeed silently
- Sensors return realistic simulated data
- Timers store configuration without hardware access

## Build Integration

The Arduino framework is built as a static library:

```cmake
add_library(arduino_framework_impl STATIC
    Arduino.cpp
    Serial.cpp
    Wire.cpp
    HardwareTimer/HardwareTimer.cpp
    RF24/RF24.cpp
    Adafruit_MPU6050/Adafruit_MPU6050.cpp
)
```

Include directories are automatically added to all targets that link against it.

## Testing Integration

For unit tests, the stubs can be replaced with mocks:
- `mock_Arduino.h/cpp` - Mockable Arduino functions
- `mock_RF24.h/cpp` - GMock-based RF24 mock
- Enables verification of exact hardware interactions

## Porting to Real Hardware

When deploying to actual embedded hardware:

1. **Remove this directory** from the build
2. **Link against real Arduino/STM32 libraries**
3. **No code changes required** - APIs are compatible
4. The same source code runs on both platforms

## Adding New Hardware

To add support for a new peripheral:

1. **Create header** in `include/` or subdirectory
2. **Implement stub** that returns safe defaults
3. **Add to CMakeLists.txt** build configuration
4. **Document behavior** in this README
5. **Create mock** in `tests/mocks/` if needed

## Example: Adding a New Sensor

```cpp
// include/NewSensor.h
class NewSensor {
public:
    bool begin() { return true; }  // Always succeeds
    float read() { return 42.0; }  // Return simulated value
};
```

```cmake
# CMakeLists.txt
add_library(arduino_framework_impl STATIC
    # ... existing files ...
    NewSensor.cpp
)
```

## Limitations

- No actual hardware I/O
- Timing is approximate (depends on OS scheduler)
- No interrupt simulation
- PWM signals are not generated
- Radio communication is simulated only

These limitations are acceptable for:
- Algorithm development
- Logic validation
- Unit testing
- Integration testing with mocked hardware

For hardware validation, deploy to actual embedded targets.

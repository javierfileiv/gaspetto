# Gaspetto - PC Emulation Environment

A cross-platform embedded systems development environment that enables PC-based emulation, testing, and debugging of Arduino-based remote-controlled car firmware.

## Overview

Gaspetto is a sophisticated embedded system that implements a remote-controlled car (Gaspetto Car) and its control box (Gaspetto Box) using an event-driven state machine architecture. This PC emulation environment allows developers to build, test, and debug the firmware on a standard PC without requiring physical hardware.

## Key Features

- **Cross-Platform Development**: Build and test embedded code on PC using CMake
- **Arduino Framework Abstraction**: PC-compatible implementations of Arduino APIs
- **Event-Driven Architecture**: State machine framework for robust, maintainable embedded software
- **Radio Communication**: RF24 (nRF24L01+) wireless communication between car and control box
- **PID Control**: Advanced movement control with IMU sensor integration
- **Unit Testing**: Comprehensive GoogleTest/GMock test suite with mocked hardware
- **Multiple Build Targets**: Support for different hardware configurations (Car, Box, NRF Sender)

## Project Structure

```
├── arduino_framework/       # PC emulation layer for Arduino APIs
├── radio_controller/        # RF24 wireless communication module
├── read_hall/              # Hall effect sensor reading (wheel encoders)
├── state_machine_framework/ # Event-driven state machine implementation
├── targets/                # Target-specific implementations
│   ├── gaspetto_car/       # Remote-controlled car firmware
│   ├── gaspetto_box/       # Control box firmware
│   └── nrf_sender/         # Test utility for radio communication
└── tests/                  # Unit tests with hardware mocks
```

## Architecture

### State Machine Framework
The system is built on a custom event-driven state machine framework that provides:
- **Active Objects**: Concurrent state machines with event queues
- **Event Queue**: Priority-based event dispatching
- **Timered Events**: Scheduled/delayed event execution
- **State Pattern**: Clean separation of state-specific behavior

### Hardware Abstraction
- **MotorControl**: Interface for motor driver control
- **IMUOrientation**: 6-axis IMU (MPU6050) for orientation sensing
- **RadioController**: nRF24L01+ wireless transceiver management
- **HardwareTimer**: PWM generation for motor control

## Building

### Prerequisites
- CMake 3.16+
- C++17 compatible compiler (GCC/Clang)
- GoogleTest (automatically fetched via FetchContent)

### Quick Start

```bash
make          # Build car (release mode)
make test     # Build and run unit tests
make help     # Show all available targets
```

### Build Targets

| Target | Description |
|--------|-------------|
| `make` / `make release` | Build Gaspetto Car (release mode) |
| `make debug` | Build Gaspetto Car (debug mode) |
| `make car` | Build Gaspetto Car |
| `make box` | Build Gaspetto Box |
| `make nrf` | Build NRF Sender |
| `make test` | Build and run unit tests |
| `make test-sanitizers` | Run tests with ASan/UBSan |
| `make coverage` | Generate code coverage report |
| `make tidy` | Run clang-tidy static analysis |
| `make format` | Format code with clang-format |
| `make clean` | Remove build artifacts |
| `make distclean` | Deep clean (including CMake cache) |

### Build Options

```bash
make BUILD_TYPE=Debug      # Build with debug symbols
make BUILD_TYPE=Release    # Build optimized (default)
make JOBS=8                # Use 8 parallel jobs
```

## Testing

The project includes comprehensive unit tests using GoogleTest and GMock:

```bash
make test                  # Build and run tests
make test-sanitizers       # Run with Address/UB sanitizers
make coverage              # Generate coverage report (opens in browser)
```

Tests cover:
- State machine transitions
- Radio communication protocols
- Movement controller behavior
- Motor control sequences
- Event queue operations

## Components

### Movement Controller
Advanced movement control system featuring:
- **Direct Motor Control**: Speed and direction control for left/right motors
- **PID-based Straight Driving**: Maintains straight line using IMU feedback
- **PID-based Turning**: Precise in-place rotation control
- **IMU Integration**: MPU6050 gyroscope for orientation tracking

### Radio Controller
Bidirectional wireless communication using nRF24L01+:
- Auto-acknowledgment with payload
- Configurable transmission power and data rate
- Event-based receive handling
- Automatic retransmission

### State Machine
Two primary states:
- **Idle State**: Low power mode, waiting for commands
- **Processing State**: Active command execution and event handling

## Hardware Support

### PC Emulation (Current Platform)
All Arduino and hardware-specific functions are stubbed for PC execution:
- Timing functions use `std::chrono`
- Hardware peripherals return simulated data
- Enables rapid development and debugging

### Embedded Targets (STM32/Arduino)
The same codebase can be compiled for:
- STM32 microcontrollers
- Arduino-compatible boards
- Any platform with Arduino framework support
## Dependencies

### Runtime
- C++17 Standard Library
- POSIX threads (for timing simulation)

### Testing
- GoogleTest/GMock (fetched automatically)

### Optional
- ccache (for faster builds)

## Contributing

When adding new features:
1. Create interfaces for hardware abstractions
2. Implement PC stubs in `arduino_framework/`
3. Add mocks in `tests/mocks/`
4. Write unit tests for new functionality
5. Ensure all tests pass before committing

## Authors

Javier FILEIV

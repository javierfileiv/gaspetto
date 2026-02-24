# Gaspetto Emu-PC

PC emulation environment for the Gaspetto remote-controlled car project. This allows development, testing, and debugging of the embedded firmware on a desktop PC without hardware.

## Overview

Gaspetto is a remote-controlled car system consisting of:

- **Gaspetto Car** - The vehicle with motors, IMU, and NRF24 radio receiver
- **Gaspetto Box** - The controller/transmitter with buttons and NRF24 radio
- **NRF Sender** - Standalone NRF24 radio test utility

The system uses a hierarchical state machine (HSM) architecture based on the Active Object pattern for event-driven control.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Active Object                          │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │  IdleState  │◄──►│ProcessState │◄──►│ PausedState │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                  │                                │
│         └──────────────────┼────────────────────────────────│
│                            ▼                                │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ EventQueue  │    │  Movement   │    │   Radio     │     │
│  │             │    │ Controller  │    │ Controller  │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

## Project Structure

```
emu-pc/
├── arduino_framework/     # Arduino API emulation for PC
│   ├── include/           # Arduino.h, Serial.h, Wire.h stubs
│   ├── HardwareTimer/     # Timer emulation
│   ├── RF24/              # NRF24L01+ radio emulation
│   └── implementations/   # Platform-specific implementations
├── state_machine_framework/
│   ├── include/           # ActiveObject, State, Event, EventQueue
│   └── src/               # Queue implementations
├── radio_controller/      # NRF24 radio abstraction layer
├── read_hall/             # Hall sensor reading (encoder)
├── targets/
│   ├── gaspetto_car/      # Car firmware
│   │   ├── movement_controller/  # Motor + IMU control with PID
│   │   └── src/           # State implementations
│   ├── gaspetto_box/      # Controller firmware
│   └── nrf_sender/        # Radio test utility
└── tests/                 # Google Test unit tests
    ├── gaspetto_car/      # Car-specific tests
    └── mocks/             # GMock implementations
```

## Prerequisites

- CMake 3.10+
- C++17 compiler (GCC 9+ or Clang 10+)
- Google Test (fetched automatically)
- Optional: ccache for faster rebuilds

## Building

### Configure and Build All Tests

```bash
mkdir build-tests && cd build-tests
cmake .. -DBUILD_TESTS=ON
cmake --build . --target utest
```


### Build All Targets Using CMakePresets

You can build all main targets and tests using CMakePresets (recommended for consistent builds):

#### Gaspetto Car
Release:
```bash
cmake --preset car-release
cmake --build build-car
```
Debug:
```bash
cmake --preset car-debug
cmake --build build-car-debug
```

#### Gaspetto Box
Release:
```bash
cmake --preset box-release
cmake --build build-box
```
Debug:
```bash
cmake --preset box-debug
cmake --build build-box-debug
```

#### NRF Sender
Release:
```bash
cmake --preset nrf-release
cmake --build build-nrf
```
Debug:
```bash
cmake --preset nrf-debug
cmake --build build-nrf-debug
```

#### Unit Tests
```bash
cmake --preset tests
cmake --build build-tests
```

All presets are defined in `CMakePresets.json` in the project root.

### VS Code Tasks

Pre-configured VS Code tasks are available:

- **CMake Build Gaspetto Car** - Build car target (Release)
- **CMake Build Gaspetto Box** - Build box target (Release)
- **CMake Build NRF Sender** - Build NRF sender (Release)
- **CMake Build UTest** - Build unit tests
- **CMake Debug Gaspetto Car** - Build car target (Debug)
- **CMake Debug Gaspetto Box** - Build box target (Debug)
- **CMake Debug NRF Sender** - Build NRF sender (Debug)
- **Run UTest** - Build and run unit tests
- **Run Pre Push test** - Full build verification

### Quick Build Commands

```bash
# Configure for car target
mkdir -p build-car && cd build-car
cmake .. -DGASPETTO_CAR=ON -DCMAKE_BUILD_TYPE=Release
cmake --build . --target gaspetto_car

# Configure for tests
mkdir -p build-tests && cd build-tests
cmake .. -DBUILD_TESTS=ON
cmake --build . --target utest

# Run tests
./tests/utest --gtest_color=yes
```

## Compile Definitions

The following compile definitions are enabled by default:

| Definition | Purpose |
|------------|---------|
| `USE_RADIO_CONTROLLER` | Enable NRF24 radio communication |
| `LOW_POWER_MODE` | Enable low power mode callbacks |
| `GASPETTO_LOG` | Enable debug logging |
| `NRF_LOG` | Enable NRF24 debug logging |

## Running Tests

```bash
cd build-tests
./tests/utest --gtest_color=yes

# Run specific test
./tests/utest --gtest_filter="MovementControllerTest.*"

# Verbose mock output
GMOCK_VERBOSE=info ./tests/utest
```

## Event System

Events drive the state machine:

| EventId | Description |
|---------|-------------|
| `ACTION` | Motor command event |
| `TIMER_ELAPSED` | Timer expiration |
| `BUTTON_PRESSED` | Physical button press |
| `RADIO_TX` | Radio transmission |

| CommandId | Description |
|-----------|-------------|
| `MOTOR_FORWARD` | Drive forward |
| `MOTOR_BACKWARD` | Drive backward |
| `MOTOR_LEFT` | Turn left |
| `MOTOR_RIGHT` | Turn right |
| `MOTOR_STOP` | Stop all motors |

## License

Proprietary - Gaspetto Project

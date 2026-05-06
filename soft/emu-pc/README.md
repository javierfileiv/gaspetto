# Gaspetto Emu-PC

PC emulation environment for the Gaspetto remote-controlled car project. This allows development, testing, and debugging of the embedded firmware on a desktop PC without hardware.

## Overview

Gaspetto is a remote-controlled car system consisting of:

- **Gaspetto Car** - The vehicle with motors, IMU, and NRF24 radio receiver
- **Gaspetto Box** - The programming controller with ADC slot sensors, NeoPixel LEDs, and NRF24 radio transmitter
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
    ├── gaspetto_box/      # Box firmware (GaspettoBox + states + pin defs)
│   └── nrf_sender/        # Radio test utility
└── tests/                 # Google Test unit tests
    ├── gaspetto_car/      # Car-specific tests
    ├── gaspetto_box/      # Box-specific tests (program builder)
    └── mocks/             # GMock implementations + Arduino stubs
```

## Prerequisites

- CMake 3.10+
- C++17 compiler (GCC 9+ or Clang 10+)
- Google Test (fetched automatically)
- Optional: ccache for faster rebuilds

## Building

### Configure and Build All Tests

```bash
cmake --preset tests
cmake --build build-tests --target utest_car
cmake --build build-tests --target utest_box
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
- **Run UTest** - Build and run unit tests (car + box)
- **Run Pre Push test** - Full build verification

### Quick Build Commands

```bash
# Gaspetto Car
cmake --preset car-release && cmake --build build-car --target gaspetto_car

# Gaspetto Box
cmake --preset box-release && cmake --build build-box --target gaspetto_box

# All tests
cmake --preset tests
cmake --build build-tests --target utest_car
cmake --build build-tests --target utest_box
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
# Car tests (29 tests)
./build-tests/tests/utest_car --gtest_color=yes

# Box tests (4 tests)
./build-tests/tests/utest_box --gtest_color=yes

# Run a specific suite
./build-tests/tests/utest_car --gtest_filter="MovementControllerTest.*"
./build-tests/tests/utest_box --gtest_filter="GaspettoBoxProgramBuilderTest.*"

# Verbose mock output
GMOCK_VERBOSE=info ./build-tests/tests/utest_car
```

### Quick Test Runner Script

A convenience script is available at the root of the `soft/` directory:

```bash
# Run all tests (uses existing build if present)
../run_tests.sh

# Force reconfigure + rebuild then run
../run_tests.sh --rebuild

# Delete build directory, reconfigure, rebuild, and run
../run_tests.sh --clean

# Build with coverage instrumentation, run tests, and generate report
../run_tests.sh --coverage
```

Coverage reports are generated under `emu-pc/build-tests-coverage/coverage/`:

- `coverage.txt` and `coverage.html` when `gcovr` is installed
- `coverage.filtered.info` and `html/index.html` when using `lcov` + `genhtml`

## Gaspetto Box PC Emulator

The `gaspetto_box` binary emulates the BlackPill hardware controller on PC. It runs the same
state machine and `GaspettoBox` business logic used on the STM32 target.

### State Machine

The box uses the same `GenericActiveObject` framework as the car. Two states drive the full programming cycle:

```
                  ┌──────────────────────────────────────┐
                  │          GaspettoBox (Active Object)  │
                  │                                       │
  ┌───────────────▼──────────┐       ┌────────────────────▼──────┐
  │         IdleState         │       │      ProcessingState       │
  │                           │       │                            │
  │  enter() →                │       │  enter() →                 │
  │    prepareForStop()       │       │    executeProgrammingCycle │
  │    SwitchToLowPowerMode() │       │      ()                    │
  │                           │       │    transitionTo(IDLE)      │
  │  on BUTTON_PRESSED →      │──────►│                            │
  │    transitionTo(PROCESSING│       │  (scan → build → send      │
  └───────────────────────────┘◄──────│   → LED feedback)          │
                                      └────────────────────────────┘
```

**IdleState** — box is in low-power STOP mode, waiting for the wake button (PB12).
**ProcessingState** — runs `executeProgrammingCycle()` synchronously then returns to IDLE:
1. Power on sensor rail → scan 20 ADC slots (5× ADS1115: 4 on `I2C1` at `0x48..0x4B`, 1 on `I2C3` at `0x4A`)
2. Build `CommandPacket` via `buildProgramFromPieces()`
3. Send packet over RF24 to the car
4. Run LED feedback animation (success / error)
5. Power off sensor rail → `transitionTo(IDLE)` → enter STOP

### Keyboard Commands

| Key | Action |
|-----|--------|
| `P` | Wake from STOP (simulates PB12 button press) |
| `D` | Load **demo** board (FORWARD / BACKWARD / LOOP with TURN_LEFT loop) |
| `E` | Load **empty** board (all slots EMPTY) |
| `O` | Load **overflow** board (all 14 main slots = LOOP_CALL → triggers overflow error) |
| `F` | Simulate **failed** RF24 transmission on next send |

### Board Layout

The box has **20 physical slots** distributed across two areas:

```
Slots  0–13  →  Main program area  (14 slots)
Slots 14–19  →  Loop sub-program   ( 6 slots)
```

Each slot holds an ADC-decoded command piece (`BoxPieceId`):

| Piece | Decoded command |
|-------|-----------------|
| `EMPTY` | (skip) |
| `FORWARD` | `MOTOR_FORWARD` |
| `BACKWARD` | `MOTOR_BACKWARD` |
| `TURN_RIGHT` | `MOTOR_TURN_RIGHT` |
| `TURN_LEFT` | `MOTOR_TURN_LEFT` |
| `STOP` | `MOTOR_STOP` |
| `LOOP_CALL` | inline-expand the loop area at this position |

### LED Signaling Guide

The Gaspetto Box uses a 3-LED NeoPixel strip (WS2812B daisy chain on pin PC_14) to provide visual feedback during operation. Each LED is assigned a specific role:

| LED | Position | Role | States |
|-----|----------|------|--------|
| LED 0 | Left | **System State** | Off (idle), Cyan (scanning), Green (success) |
| LED 1 | Center | **RF Radio Status** | Off (idle), White (transmitting), Red (RF error) |
| LED 2 | Right | **Build/Program Errors** | Off (idle), Amber (empty board), Red (overflow/compile error) |

#### Animation Sequences

**Success Cycle** (Program scanned & sent successfully):
1. LED 0 → Cyan bouncing pattern (left ↔ right) for 4 seconds
2. LED 0 → Green cascade (LED0 → LED1 → LED2) lights in sequence
3. All LEDs hold final state (green) for **60 seconds**
4. Transition to IDLE (all LEDs off)

**Build Error** (Overflow or compile failure detected):
1. LED 2 → Red blinks 3 times (500ms on/off)
2. LED 2 holds red for **60 seconds**
3. Transition to IDLE (LED 2 off)

**Empty Board Error** (No valid commands found):
1. LED 2 → Amber blinks 2 times (500ms on/off)
2. LED 2 holds amber for **60 seconds**
3. Transition to IDLE (LED 2 off)

**RF Transmission Error** (nRF24 send failed):
1. LED 1 → Red blinks 3 times (500ms on/off)
2. LED 0 → Green (confirms board scanned OK)
3. LED 1 holds red + LED 0 remains green for **60 seconds**
4. Transition to IDLE (all LEDs off)

**Idle Mode**:
- All LEDs off
- Box in low-power STOP mode (awakened by PB12 button)

### Program Build Algorithm

`GaspettoBox::buildProgramFromPieces()` (pure static, also tested in `utest_box`):

1. Iterate main slots 0–13, appending each non-EMPTY piece as a command.
2. On `LOOP_CALL`: inline all non-EMPTY loop slots 14–19 at that position.
3. `LOOP_CALL` inside the loop area → **error**.
4. If the output exceeds `BOX_MAX_PROGRAM_COMMANDS` (31) → **error** (overflow).
5. If all slots are EMPTY → **empty board** (not sent).

The resulting `CommandPacket` (32 bytes, packed) is sent via RF24 to the car.

### Running the Box Emulator

```bash
cmake --preset box-release && cmake --build build-box --target gaspetto_box
./build-box/targets/gaspetto_box/gaspetto_box
```

## IMU CSV Playback In PC Emulation

The emulated `Adafruit_MPU6050` can replay raw IMU samples from a CSV file.
This is useful for deterministic debugging and regression tests.

Enable it with environment variables before launching `gaspetto_car`:

```bash
IMU_CSV=/absolute/path/to/imu_samples.csv IMU_LOOP=1 ./build-car/targets/gaspetto_car/gaspetto_car
```

- `IMU_CSV`: CSV file path. If missing or unreadable, the default built-in stub values are used.
- `IMU_LOOP`: Optional. `1` (default) loops when the end of file is reached. `0` holds the last sample.

Supported row formats:

```text
ax,ay,az,gx,gy,gz
ax,ay,az,gx,gy,gz,tempC
t,ax,ay,az,gx,gy,gz
t,ax,ay,az,gx,gy,gz,tempC
```

Notes:

- Lines starting with `#` are ignored.
- A single header line (for example `ax,ay,az,gx,gy,gz,tempC`) is allowed.
- Units are expected to match the Adafruit API: acceleration in m/s^2 and gyro in rad/s.

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

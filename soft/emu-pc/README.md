# Gaspetto Emu-PC

PC emulation environment for Gaspetto. Build and run the same car and box firmware logic on a desktop machine for fast iteration, interactive testing, and GoogleTest-based unit tests.

## Overview

Gaspetto consists of:

- **Gaspetto Car** — motors, IMU, NRF24 receiver
- **Gaspetto Box** — ADC slot sensors, NeoPixel LEDs, NRF24 transmitter
- **NRF Sender** — standalone radio test utility

Both car and box use the Active Object pattern with a small state machine (`IdleState` ↔ `ProcessingState`).

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Active Object                          │
│  ┌─────────────┐    ┌─────────────┐                        │
│  │  IdleState  │◄──►│ProcessState │                        │
│  └─────────────┘    └─────────────┘                        │
│         │                  │                                │
│         └──────────────────┼────────────────────────────────│
│                            ▼                                │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │ EventQueue  │    │  Movement   │    │   Radio     │     │
│  │             │    │ Controller  │    │ Controller  │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

## Project structure

```
emu-pc/
├── arduino_framework/     # Arduino API emulation for PC
├── state_machine_framework/
├── radio_controller/      # NRF24 abstraction
├── targets/
│   ├── gcar/      # Car firmware
│   │   ├── movement_controller/  # Motor + IMU control with PID
│   │   └── src/           # State implementations
    ├── gaspetto_box/      # Box firmware (GaspettoBox + states + pin defs)
│   └── nrf_sender/        # Radio test utility
└── tests/                 # Google Test unit tests
    ├── gcar/      # Car-specific tests
    ├── gaspetto_box/      # Box-specific tests (program builder)
    └── mocks/             # GMock implementations + Arduino stubs
```

## Prerequisites

- CMake 3.10+
- C++17 compiler (GCC 9+ or Clang 10+)
- Google Test (fetched automatically by CMake)
- Optional: `gcovr` for coverage, `ccache` for faster rebuilds

All `cmake --preset` commands below are run from this directory (`soft/emu-pc/`). Presets are defined in `CMakePresets.json`.

## Building

### Unit tests

```bash
cmake --preset tests
cmake --build build-tests --target utest_car
cmake --build build-tests --target utest_box
```

### PC emulators

```bash
# Gaspetto Car
cmake --preset car-release && cmake --build build-car --target gcar

# Box
cmake --preset box-release && cmake --build build-box --target gaspetto_box
```

Debug presets (`car-debug`, `box-debug`, `nrf-debug`) and NRF sender targets follow the same pattern.

### VS Code tasks

Pre-configured tasks include **CMake Build Gaspetto Car/Box**, **CMake Build UTest**, **Run UTest**, and **Run Pre Push test**.

## PC vs firmware build flags

CMake and PlatformIO do not enable the same defines today. This matters when behavior differs between emulator and hardware.

| Definition | emu-pc (CMake) | Car PIO | Box PIO |
|------------|----------------|---------|---------|
| `USE_RADIO_CONTROLLER` | **on** | off (commented) | on (via radio lib) |
| `LOW_POWER_MODE` | **on** | off | off |
| `GASPETTO_LOG` | **on** | on | on |
| `GASPETTO_LOG_OVER_NRF24` | off | off (optional) | off (optional) |
| USB CDC + `dfu_upload.py` | n/a | partial | **on** |

**Implication:** the PC car emulator always listens on NRF24. The car PIO build currently schedules a timer-based motor demo unless you uncomment `-DUSE_RADIO_CONTROLLER` in `GCar_pio/platformio.ini`.

Low-power STOP mode is emulated on PC (`SwitchToLowPowerMode`). On STM32 PIO builds, `enterLowPowerMode()` is still a stub (`delay(100)`).

## Compile definitions (emu-pc defaults)

| Definition | Purpose |
|------------|---------|
| `USE_RADIO_CONTROLLER` | NRF24 radio communication |
| `LOW_POWER_MODE` | Low-power mode callbacks |
| `GASPETTO_LOG` | Debug logging to Serial |
| `NRF_LOG` | NRF24 debug logging |

Box-only tuning defines (`GASPETTO_ADC_THRESHOLD_*`, `GASPETTO_I2C_CLOCK_HZ`, etc.) are documented as comments in `soft/pio/GaspettoBox_pio/platformio.ini`.

## Running tests

```bash
# from soft/emu-pc/
./build-tests/tests/utest_car --gtest_color=yes
./build-tests/tests/utest_box --gtest_color=yes

# filter a suite
./build-tests/tests/utest_car --gtest_filter="MovementControllerTest.*"
./build-tests/tests/utest_box --gtest_filter="GaspettoBoxProgramBuilderTest.*"

# list all tests
./build-tests/tests/utest_car --gtest_list_tests
```

### Quick test runner

`soft/run_tests.sh` wraps configure, build, and run:

```bash
# from repository root
soft/run_tests.sh

# from soft/emu-pc/
../run_tests.sh

# options
../run_tests.sh --rebuild
../run_tests.sh --clean
../run_tests.sh --coverage
```

Coverage output: `build-tests-coverage/coverage/coverage.txt` and `coverage.html`.

## Radio protocol

Two payload shapes share the same NRF24 pipe (32-byte max):

| Packet | Size | Used by |
|--------|------|---------|
| `EventPacket` | 2 bytes | Single command (`eventId` + `CommandId`) |
| `CommandPacket` | 32 bytes | Full program from box (`count` + up to 31 commands) |

The car's `RadioController` distinguishes them heuristically from buffer content. A dedicated packet-type byte would be more robust — treat mis-decoding as a known limitation.

## Gaspetto Car PC emulator

```bash
cmake --preset car-release && cmake --build build-car --target gaspetto_car
./build-car/targets/gaspetto_car/gaspetto_car
```

With `USE_RADIO_CONTROLLER` (default on PC), keyboard commands simulate radio input:

| Key | Action |
|-----|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Turn left |
| `D` | Turn right |
| `X` | Stop |

### IMU CSV playback

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

Enable it with environment variables before launching `gcar`:

```bash
IMU_CSV=/absolute/path/to/imu_samples.csv IMU_LOOP=1 ./build-car/targets/gcar/gcar
```

- `IMU_CSV` — path to CSV; falls back to built-in stub values if missing
- `IMU_LOOP` — `1` (default) loops at EOF; `0` holds last sample

Supported row formats:

```text
ax,ay,az,gx,gy,gz
ax,ay,az,gx,gy,gz,tempC
t,ax,ay,az,gx,gy,gz
t,ax,ay,az,gx,gy,gz,tempC
```

Lines starting with `#` and a single header row are allowed. Units match the Adafruit API: acceleration in m/s², gyro in rad/s.

## Gaspetto Box PC emulator

```bash
cmake --preset box-release && cmake --build build-box --target gaspetto_box
./build-box/targets/gaspetto_box/gaspetto_box
```

### State machine

```
                  ┌──────────────────────────────────────┐
                  │          GaspettoBox (Active Object)  │
  ┌───────────────▼──────────┐       ┌────────────────────▼──────┐
  │         IdleState         │       │      ProcessingState       │
  │  enter() → low power      │       │  scan → build → send       │
  │  BUTTON_PRESSED →         │──────►│  → LED feedback → IDLE     │
  └───────────────────────────┘◄──────└────────────────────────────┘
```

**ProcessingState** runs `executeProgrammingCycle()` synchronously:

1. Power sensor rail → scan 20 ADC slots (5× ADS1115)
2. Build `CommandPacket` via `buildProgramFromPieces()`
3. Send over RF24
4. LED feedback animation
5. Power off sensor rail → return to IDLE / STOP

### Keyboard commands

| Key | Action |
|-----|--------|
| `P` | Wake from STOP (simulates PB12) |
| `D` | Load **demo** board |
| `E` | Load **empty** board |
| `O` | Load **overflow** board (14× `LOOP_CALL`) |
| `F` | Simulate failed RF24 transmission |

### Board layout

```
Slots  0–13  →  Main program area  (14 slots)
Slots 14–19  →  Loop sub-program   ( 6 slots)
```

| Piece (`BoxPieceId`) | Motor command |
|----------------------|---------------|
| `EMPTY` | (skip) |
| `FORWARD` | `MOTOR_FORWARD` |
| `BACKWARD` | `MOTOR_BACKWARD` |
| `TURN_RIGHT` | `MOTOR_TURN_RIGHT` |
| `TURN_LEFT` | `MOTOR_TURN_LEFT` |
| `STOP` | `MOTOR_STOP` |
| `LOOP_CALL` | Inline-expand loop slots 14–19 |

### LED signaling

3-LED NeoPixel strip (WS2812B on `PC_14`):

| LED | Role | Idle | Active states |
|-----|------|------|---------------|
| 0 | System | Off | Cyan (scanning), Green (success) |
| 1 | RF | Off | White (TX), Red (RF error) |
| 2 | Build errors | Off | Amber (empty), Red (overflow) |

After success or error animations, the active LED holds for **60 seconds** before returning to idle.

### Program build algorithm

`GaspettoBox::buildProgramFromPieces()` (tested in `utest_box`):

1. Iterate main slots 0–13; append non-`EMPTY` pieces.
2. On `LOOP_CALL`, inline non-`EMPTY` loop slots 14–19.
3. `LOOP_CALL` inside the loop area → error.
4. More than 31 output commands → overflow error.
5. All slots empty → empty board (not transmitted).

## Event system

| `EventId` | Description |
|-----------|-------------|
| `ACTION` | Motor command |
| `TIMER_ELAPSED` | Timer expiration |
| `BUTTON_PRESSED` | Physical button |
| `RADIO_TX` | Radio transmission |

| `CommandId` | Description |
|-------------|-------------|
| `MOTOR_FORWARD` | Drive forward |
| `MOTOR_BACKWARD` | Drive backward |
| `MOTOR_TURN_RIGHT` | Turn right |
| `MOTOR_TURN_LEFT` | Turn left |
| `MOTOR_STOP` | Stop motors |
| `QUEUE_CLEAR` | Clear queued commands and stop |

## Hardware test firmware

These PIO projects live alongside production firmware under `soft/pio/`:

| Project | Purpose |
|---------|---------|
| `arduino_box_hw_test` | ADS1115, NeoPixel, NRF bring-up |
| `arduino_car_hw_test` | Motors, IMU bring-up |
| `arduino_straight_drive` | PWM threshold + PID straight-drive tuning |
| `mpu_plot` | IMU plotting utility |

See the root [README.md](../../README.md) for build and flash commands.

## License

Proprietary — Gaspetto Project. See root README.

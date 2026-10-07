# Gaspetto

Gaspetto is an open-source robotics project inspired by Cubetto, with:

- embedded firmware targets (car, box, NRF sender)
- a PC emulation layer for fast iteration and unit testing
- CI pipelines for build, tests, and coverage

[![Gaspetto CI](https://github.com/javierfileiv/gaspetto/actions/workflows/ci.yml/badge.svg?event=push)](https://github.com/javierfileiv/gaspetto/actions/workflows/ci.yml)
[![Coverage](https://img.shields.io/endpoint?url=https://javierfileiv.github.io/gaspetto/coverage-badge.json&cacheSeconds=300)](https://javierfileiv.github.io/gaspetto/coverage/coverage.html)

## How It Works

Gaspetto is a physical programming game: instead of tapping blocks on a
screen, the player builds a program out of tangible pieces on the Box,
and a small robot car executes it.

- **GBox** — the programming board. The player arranges command pieces
  on the sensor slots and presses the button. The Box reads the board,
  builds the program and beams it to the car.
- **GCar** — the robot. It receives the program and runs it step by
  step, driving forward, backward and turning until the program ends.

### The Pieces

- `FORWARD`, `BACKWARD`, `TURN_RIGHT`, `TURN_LEFT` — movement commands
- `LOOP_CALL` — replays the loop zone right where it sits

The board has two zones: the main zone and the loop zone. Pieces in
the loop zone are not run directly; each LOOP_CALL piece in the main
zone replays them at that point in the program. With a full loop zone,
up to three LOOP_CALLs fit in one program. There is no STOP piece —
the car stops by itself when the program ends.

### The Lights

Three LEDs on the Box speak a small language:

- on startup, the left LED lights green when every part of the Box is
  healthy, or red if something is wrong
- while the board is scanned, a cyan dot bounces across the three LEDs
- when the program is accepted, green lights flow from left to right
  and all three stay green while the car runs
- if the pieces make an invalid program, the right LED blinks red and
  stays lit
- if the board is empty, the right LED blinks amber and stays lit
- if the car could not be reached, the center LED blinks red and stays
  lit next to the green left LED

In all end cases the pattern holds for about a minute, then the Box
goes quiet to save power until the next button press.

### A Round Of Play

1. Lay pieces on the board
2. Press the Box button
3. The Box scans the board, builds the program and sends it to the car
4. The car executes the moves in order
5. Pressing the button again stops the car and empties the program

## Repository Layout

- `soft/emu-pc`: host-side CMake project with GoogleTest-based tests and coverage
- `soft/pio`: PlatformIO firmware projects (`GCar_pio`, `GBox_pio`, `NrfSender_pio`, hardware test apps)
- `tools`: helper scripts used locally and in CI
- `.github/workflows`: CI workflows

## Prerequisites

- Linux/macOS shell with `bash`
- `cmake`
- C++ toolchain (`gcc/g++` or `clang`)
- `python3`
- `gcovr` (for coverage)
- `platformio` (for PIO firmware builds)

## Common Commands

Run from repository root.

- Build all local helper scripts:

	```bash
	./tools/build-all.sh
	```

- Build emu-pc targets:

	```bash
	./tools/build-gaspetto-pc.sh
	```

- Build and run emu-pc unit tests:

	```bash
	./tools/build-gaspetto-utest.sh
	```

- Build all PlatformIO targets:

	```bash
	./tools/build-gaspetto-pio.sh
	```

- Build tests with coverage and generate reports:

	```bash
	./tools/coverage-gaspetto-utest
	```

## Tuning

The `arduino_straight_drive` project (`soft/pio/arduino_straight_drive/`) provides two programs for PID tuning on real hardware:

| Target | Board | Description |
|--------|-------|-------------|
| `arduino_straight_drive` | BlackPill F411CE + MPU6050 + DRV8871 + NRF24L01 | Receiver: runs PID on the robot |
| `motor_test_sender` | Arduino Uno + NRF24L01 | Sender: sends commands via radio |

### Flash

```bash
cd soft/pio/arduino_straight_drive

# Receiver (robot)
pio run -e arduino_straight_drive -t upload

# Sender (Arduino Uno)
pio run -e motor_test_sender -t upload
```

### Commands

All commands are sent from the sender as text (e.g. `w50`). Lowercase sends immediately, uppercase adds to the program queue (send with `*`).

```
--- Direct motor (PWM 500ms) ---
  L<v>  Left motor
  R<v>  Right motor
  B<v>  Both motors
  B     Debug telemetry
  M     Full test all dirs

--- PID movement ---
  W<v>  Forward
  S<v>  Backward
  A<v>  Turn left 90 deg
  D<v>  Turn right 90 deg
  X     Stop all
  Z     Zero yaw
  M<v>  Set movement timeout (ms)

--- PID tuning ---
  K<v>  Kp (K50 = 0.05)
  I<v>  Ki (I100 = 0.10)
  V<v>  Kd (V50 = 0.05)
  O<v>  Trim offset (±100)

--- Program ---
  *     Send program queue
  -     Clear program queue
  T     Toggle telemetry display
```

### Telemetry

Auto-sent every 500ms during movement, shown on the sender (throttled to 1/10). Press `B` (or space on the receiver serial) for an immediate debug dump.

Format:
```
3245 W50 Y=2.3 e=0.5 Kp=2.000 Ki=0.010 Kd=0.010 O=0
```

| Field | Meaning |
|-------|---------|
| `3234` | Timestamp (millis) |
| `W50` | State (IDL/FWD/BWD/TL/TR) + speed |
| `Y` | Current yaw (degrees) |
| `e` | PID error |
| `Kp/Ki/Kd` | PID gains |
| `O` | Trim offset |

### Shared PID (GCar)

The PID controller (`MovementController`), motor driver (`MotorControl`), and IMU (`IMUOrientation`) are shared between `motor_test_main` and the GCar firmware via symlinks. Tuning values found with the test program can be directly applied to GCar — the behavior is identical.

## Coverage Outputs

After running `./tools/coverage-gaspetto-utest`, reports are generated in:

- `soft/emu-pc/build-tests-coverage/coverage/coverage.txt`
- `soft/emu-pc/build-tests-coverage/coverage/coverage.html`

## Coverage Artifacts In CI

Yes, coverage is already published as workflow artifacts.

In `.github/workflows/ci.yml`, the `coverage` job uploads:

- artifact name: `emu-pc-coverage`
- artifact path: `soft/emu-pc/build-tests-coverage/coverage`

You can download it from the GitHub Actions run page:

1. Open the workflow run.
2. Go to the Artifacts section.
3. Download `emu-pc-coverage`.

The job also publishes a coverage summary in the run summary page.

On pushes to `main`, CI also publishes a live coverage page and dynamic badge via GitHub Pages:

- coverage page: https://javierfileiv.github.io/gaspetto/coverage/coverage.html
- dynamic badge endpoint source: https://javierfileiv.github.io/gaspetto/coverage-badge.json

## Notes

- `tools/build-all.sh` executes all `*.sh` scripts in `tools/` except itself.
- For module-specific details, check the README files inside each project folder (for example `soft/emu-pc/`).

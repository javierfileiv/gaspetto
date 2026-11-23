# Gaspetto

Gaspetto is an open-source robotics project inspired by Cubetto, with:

- embedded firmware targets (car, box)
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
- `soft/pio`: PlatformIO firmware projects
	- `GCar_pio`, `GBox_pio`: robot and box firmware
	- `GCar_boot`: the robot bootloader, flashed by hand
	- `NrfLink_pio`: radio link test tool for Arduino Uno + NRF24L01
	- hardware test apps
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

## Flashing the robot over DFU

`GCar_pio` uploads through the STM32 ROM bootloader, and the request that
switches the board into it travels over the nRF24 link: the car has no USB
command channel, so nothing can be typed on a serial port to wake the
bootloader.

- `pio run -e gcar -t upload` runs the whole sequence itself: it opens
  NrfLink's serial port, waits for the Uno to boot, writes `f`, waits for the
  `0483:df11` device, then calls `dfu-util`
- `f` typed in NrfLink does the same by hand and works from any mode: the car
  stops its motors and re-enumerates about a second later
- `GCAR_NRF_PORT=/dev/ttyUSBx` overrides the default NrfLink port when the
  board is not the usual CH340

A failed request costs nothing: the application was never touched, so a RESET
boots it again through the bootloader. BOOT0 plus RESET remains available as a
last resort.

## The serial link tool

`tools/gaspetto_link.py` is the PC side of the radio link. With no option it is
a plain console on the port, like `tio`: it does not reset the board and does
not send anything. `Ctrl-]` leaves.

```bash
./tools/gaspetto_link.py --port /dev/ttyUSB0   # NrfLink console
./tools/gaspetto_link.py --port /dev/ttyACM0   # box console
```

With `--ota` it rebuilds the car firmware, streams it over the radio through
the relay on the port, the NrfLink or the GBox, and then stays on the console,
so the car's boot lines can be watched without opening another terminal. `--no-build` sends the binary
already built in `GCar_pio/.pio/build/gcar/`, and `--logs N` reads the port for
N seconds and leaves instead of handing the console over. `GASPETTO_LINK_PORT`
overrides the default port; the older `GCAR_NRF_PORT` still works.

With `--box` (or `--dfu`) the tool flashes the box itself: it rebuilds
`GBox_pio`, sends `dfu` on the box's serial port, waits for the `0483:df11`
device, then runs `dfu-util`. The box enters the STM32 ROM bootloader on that
serial command, so no tio session is needed. `--no-build` sends the box binary
already built in `GBox_pio/.pio/build/blackpill_f411ce/`.

### Memory layout

The board carries the bootloader alone in sector 0, the validation record
alone in sector 1, and the application in the next 96 kB (sectors 2 to 4), so an
application upload can never erase the bootloader or the record that proves the
image is intact:

| Address      | Size   | Content                                      |
|--------------|--------|----------------------------------------------|
| `0x08000000` | 16 kB  | bootloader (`soft/pio/GCar_boot`)            |
| `0x08004000` | 16 kB  | validation record, alone in its sector       |
| `0x08008000` | 96 kB  | application (`soft/pio/GCar_pio`)            |
| `0x08020000` | 128 kB | staging area for a radio update              |

The bootloader checks that an image is present, that the validation record
matches it, and jumps to it. It is flashed by hand, and again only when it
changes: put the board in DFU, then `pio run -d soft/pio/GCar_boot -t upload`.
Application uploads go to `0x08008000`, which `tools/gcar_dfu_upload.py` already
uses. Each upload writes the record too: the image length and its CRC32.

When the record carries the copy-pending flag, the bootloader copies the
staging area into the application slot first, verifies the CRC on the copied
bytes, and clears the flag before jumping. A failed copy leaves the flag set,
so the next boot retries it.

### When the bootloader refuses to start

It has no serial port and no radio, so it says why with the LED on PC13:

| Blinks | Meaning                                                       |
|--------|---------------------------------------------------------------|
| 1      | no image at all: the slot is erased or its vectors are nonsense |
| 2      | the validation record is missing or unreadable                |
| 3      | the record is there but the CRC does not match                |
| 4      | the copy from the staging area failed, the next boot retries   |

A refusal code repeats forever. After a successful copy the bootloader blinks
three times instead, then starts the application: the pattern stops, which is
how you tell the end of an update from a failure.

The third code is the one worth provoking on a bench, because it proves the
check actually runs: `./tools/gcar_dfu_upload.py --bad-crc` flashes the
application with a deliberately wrong CRC, so the board must blink three times
and never jump. Reflash without the flag to bring it back. `--no-record` leaves
the record untouched and exercises the other refusal path.

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
| `e` | Yaw error (`targetYaw - yaw`) |
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

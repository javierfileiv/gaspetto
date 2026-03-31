
# Gaspetto — Project Overview

Gaspetto is an open-source robotics project focused on a remote-controlled car, its controller, and a suite of development and testing tools. It includes embedded firmwares, PC emulation tools and sensor visualization utilities.

## Project Structure

- **hardware/**, **kicad/**, **FreeCad/**: Hardware files, electronic schematics, CAD.
- **soft/**: All software code (see details below).
- **documentation/**: Technical docs and guides.
- **tools/**: Automation scripts and utilities.

---

## Folders in soft/

### soft/emu-pc/
PC emulation environment for developing and testing Gaspetto firmware without hardware. Uses CMake, C++17, Google Test. Simulates the car, controller, and NRF24 radio.
**Key subfolders:**
- `arduino_framework/`: Arduino API simulation for PC.
- `state_machine_framework/`: Hierarchical state machine implementation.
- `radio_controller/`, `read_hall/`: Radio and sensor abstractions.
- `targets/`: Simulated firmwares (car, box, nrf_sender).
- `tests/`: Google Test unit tests.

### soft/pio/
PlatformIO projects for deployment on microcontrollers (STM32, Arduino, etc.).
- `GaspettoCar_pio/`: Main car firmware.
- `NrfSender_pio/`: NRF24 radio test utility.
- `arduino_hw_test/`, `arduino_straight_drive/`: Hardware tests and driving routines.
- `mpu_plot/`: Python tool for real-time visualization of MPU6050 sensor data (see README.txt for usage).

### soft/Tests/
Unit and integration tests for various modules:
- `NRFBridge/`: NRF24 radio bridge.
- `arduino-tests/`: Tests for Arduino platforms.
- `stm_simple_dc_control/`, `stm_test_pwm/`: DC motor and PWM control on STM32.

### soft/sender_test/
Example Arduino sketch for testing NRF24 radio sending (`sender_test.ino`).

### soft/schematic.py
Python script to programmatically generate KiCad 8.0-compatible schematics.

---

## Further Information

- See each subfolder's README for build and usage instructions.
- For PC emulation: see soft/emu-pc/README.md.
- For MPU6050 visualization: see soft/pio/mpu_plot/README.txt.

# Gaspetto Project

[![Gaspetto CI](https://github.com/javierfileiv/gaspetto/actions/workflows/ci.yml/badge.svg?event=push)](https://github.com/javierfileiv/gaspetto/actions/workflows/ci.yml)

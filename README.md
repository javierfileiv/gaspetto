# Gaspetto

Gaspetto is an open-source robotics project inspired by Cubetto, with:

- embedded firmware targets (car, box, NRF sender)
- a PC emulation layer for fast iteration and unit testing
- CI pipelines for build, tests, and coverage

[![Gaspetto CI](https://github.com/javierfileiv/gaspetto/actions/workflows/ci.yml/badge.svg?event=push)](https://github.com/javierfileiv/gaspetto/actions/workflows/ci.yml)
[![Coverage](https://img.shields.io/endpoint?url=https://javierfileiv.github.io/gaspetto/coverage-badge.json)](https://javierfileiv.github.io/gaspetto/coverage/coverage.html)

## Repository Layout

- `soft/emu-pc`: host-side CMake project with GoogleTest-based tests and coverage
- `soft/pio`: PlatformIO firmware projects (`GaspettoCar_pio`, `GaspettoBox_pio`, `NrfSender_pio`, hardware test apps)
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

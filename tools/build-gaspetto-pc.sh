#!/usr/bin/env bash

# CI build-and-smoke script for emu-pc targets.
# It builds release+debug targets and runs each binary briefly in headless mode.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
EMU_PC_DIR="$PROJECT_ROOT/soft/emu-pc"
NUM_CORES="$(nproc)"
SMOKE_TIMEOUT_SECONDS="${SMOKE_TIMEOUT_SECONDS:-5}"

echo -e "${BLUE}Building Gaspetto emu-pc targets (release + debug)${NC}"
echo "Project root: $PROJECT_ROOT"
echo "Emu-pc dir  : $EMU_PC_DIR"
echo "Parallel jobs: $NUM_CORES"

if ! command -v timeout >/dev/null 2>&1; then
  echo -e "${RED}Required command 'timeout' not found.${NC}" >&2
  exit 1
fi

configure_and_build() {
  local preset="$1"
  local build_dir="$2"
  local target="$3"

  echo -e "${YELLOW}Configuring preset: ${preset}${NC}"
  cmake --preset "$preset" -S "$EMU_PC_DIR"

  echo -e "${YELLOW}Building target: ${target} (${build_dir})${NC}"
  cmake --build "$EMU_PC_DIR/$build_dir" --parallel "$NUM_CORES" --target "$target"
}

smoke_run() {
  local label="$1"
  local bin_path="$2"

  echo -e "${YELLOW}Smoke-running ${label} for ${SMOKE_TIMEOUT_SECONDS}s: ${bin_path}${NC}"
  set +e
  timeout --signal=INT "${SMOKE_TIMEOUT_SECONDS}" "$bin_path"
  local exit_code=$?
  set -e

  # 0 = process exited itself, 124/130 = timed out/interrupted after proving startup.
  if [[ $exit_code -eq 0 || $exit_code -eq 124 || $exit_code -eq 130 ]]; then
    echo -e "${GREEN}${label} smoke-run OK (exit ${exit_code})${NC}"
  else
    echo -e "${RED}${label} smoke-run FAILED (exit ${exit_code})${NC}" >&2
    exit $exit_code
  fi
}

configure_and_build "car-release" "build-car" "gcar"
configure_and_build "car-debug" "build-car-debug" "gcar"
configure_and_build "box-release" "build-box" "gaspetto_box"
configure_and_build "box-debug" "build-box-debug" "gaspetto_box"
configure_and_build "nrf-release" "build-nrf" "nrf_sender"
configure_and_build "nrf-debug" "build-nrf-debug" "nrf_sender"

smoke_run "Gaspetto Car (Release)" "$EMU_PC_DIR/build-car/targets/gcar/gcar"
smoke_run "Gaspetto Car (Debug)" "$EMU_PC_DIR/build-car-debug/targets/gcar/gcar"
smoke_run "Gaspetto Box (Release)" "$EMU_PC_DIR/build-box/targets/gaspetto_box/gaspetto_box"
smoke_run "Gaspetto Box (Debug)" "$EMU_PC_DIR/build-box-debug/targets/gaspetto_box/gaspetto_box"
smoke_run "NRF Sender (Release)" "$EMU_PC_DIR/build-nrf/targets/nrf_sender/nrf_sender"
smoke_run "NRF Sender (Debug)" "$EMU_PC_DIR/build-nrf-debug/targets/nrf_sender/nrf_sender"

echo -e "${GREEN}All emu-pc builds and smoke-runs succeeded.${NC}"

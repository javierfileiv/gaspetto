#!/bin/bash

# Unified PlatformIO build script that aborts on first failure.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

START_TS=$(date +%s)
CURRENT_STEP="(initialization)"

trap 'echo -e "${RED}ERROR: Build failed at step: ${CURRENT_STEP}${NC}" >&2' ERR

echo -e "${BLUE}Building Gaspetto (PlatformIO multi-project)${NC}"
echo -e "Root: $PROJECT_ROOT"

if ! command -v pio &>/dev/null; then
    echo -e "${RED}PlatformIO (pio) not found in PATH.${NC}" >&2
    exit 1
fi

build_step() {
    local path="$1"; shift
    local label="$1"; shift
    CURRENT_STEP="$label"
    echo -e "${YELLOW}==> $label (${path})${NC}"

    if [[ ! -f "$path/platformio.ini" ]]; then
        echo -e "${YELLOW}Skipping $label: no platformio.ini in $path${NC}"
        return 0
    fi

    pushd "$path" >/dev/null
    pio run "$@"
    popd >/dev/null
    echo -e "${GREEN}✔ $label OK${NC}"
}

ensure_dir() {
    local path="$1"
    if [[ ! -d "$path" ]]; then
        echo -e "${RED}Missing directory: $path${NC}" >&2
        exit 1
    fi
}

# List of builds (path label [extra pio args...])
declare -a BUILD_PATHS=(
    "$PROJECT_ROOT/soft/pio/GCar_pio"
    "$PROJECT_ROOT/soft/pio/GBox_pio"
    "$PROJECT_ROOT/soft/pio/NrfSender_pio"
    "$PROJECT_ROOT/soft/pio/arduino_straight_drive"
    "$PROJECT_ROOT/soft/pio/arduino_box_hw_test"
    "$PROJECT_ROOT/soft/pio/arduino_car_hw_test"
    "$PROJECT_ROOT/soft/pio/mpu_plot"
)

declare -a BUILD_LABELS=(
    "GCar"
    "GBox"
    "NrfSender"
    "Arduino Straight Drive"
    "Arduino Box HW Test"
    "Arduino Car HW Test"
    "MPU Plot"
)

for path in "${BUILD_PATHS[@]}"; do
    ensure_dir "$path"
done

for i in "${!BUILD_PATHS[@]}"; do
    build_step "${BUILD_PATHS[$i]}" "${BUILD_LABELS[$i]}"
done

END_TS=$(date +%s)
ELAPSED=$((END_TS-START_TS))
echo -e "${GREEN}All builds succeeded in ${ELAPSED}s${NC}"

# Optional cleanup of PlatformIO build directories
if [[ "${CLEAN_BUILD:-1}" == "1" ]]; then
    echo -e "${BLUE}Cleaning build artifacts (.pio) directories...${NC}"
    while IFS= read -r -d '' dir; do
        if [[ -d "$dir/.pio" ]]; then
            echo -e "  Removing $dir/.pio"
            rm -rf "$dir/.pio" || echo -e "${YELLOW}Warning: failed to remove $dir/.pio${NC}" >&2
        fi
    done < <(find "$PROJECT_ROOT/soft/pio" -mindepth 1 -maxdepth 1 -type d -print0)
    echo -e "${GREEN}Cleanup complete.${NC}"
else
    echo -e "${YELLOW}Skipping cleanup (set CLEAN_BUILD=1 to enable).${NC}"
fi

exit 0

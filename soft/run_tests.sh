#!/usr/bin/env bash
# Run all PC-runnable unit tests for the Gaspetto project.
# Usage: ./run_tests.sh [--rebuild] [--clean] [--coverage] [--help]
#   --rebuild   Force a full CMake reconfigure + rebuild before running
#   --clean     Delete the build directory then reconfigure + rebuild
#   --coverage  Build tests with coverage instrumentation and generate a report
#   --help      Show this help message and exit

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EMU_PC_DIR="$SCRIPT_DIR/emu-pc"
DEFAULT_BUILD_DIR="$EMU_PC_DIR/build-tests"
COVERAGE_BUILD_DIR="$EMU_PC_DIR/build-tests-coverage"
BUILD_DIR="$DEFAULT_BUILD_DIR"
NUM_CORES="$(nproc)"
COVERAGE_FLAGS="--coverage -O0 -g"

generate_coverage_report() {
    local coverage_dir="$BUILD_DIR/coverage"
    mkdir -p "$coverage_dir"

    if command -v gcovr >/dev/null 2>&1; then
        echo "--- Generating coverage report with gcovr ---"
        local coverage_filters=(
            --filter "$EMU_PC_DIR/targets"
            --filter "$EMU_PC_DIR/radio_controller"
            --filter "$EMU_PC_DIR/shared_types"
            --filter "$EMU_PC_DIR/state_machine_framework"
        )

        gcovr \
            --root "$EMU_PC_DIR" \
            --object-directory "$BUILD_DIR" \
            --gcov-ignore-errors=no_working_dir_found \
            --exclude-throw-branches \
            --exclude-unreachable-branches \
            --exclude-directories "$BUILD_DIR/CMakeFiles" \
            --exclude ".*CMakeCCompilerId.*" \
            --exclude ".*CMakeCXXCompilerId.*" \
            --exclude "$EMU_PC_DIR/arduino_framework/.*" \
            --exclude "$EMU_PC_DIR/state_machine_framework/include/.*" \
            --exclude "$EMU_PC_DIR/shared_types/include/.*" \
            --exclude "$EMU_PC_DIR/tests/.*" \
            --exclude "$EMU_PC_DIR/build-.*/.*" \
            "${coverage_filters[@]}" \
            --print-summary \
            --txt "$coverage_dir/coverage.txt" \
            --html-details "$coverage_dir/coverage.html"
        echo "Coverage summary: $coverage_dir/coverage.txt"
        echo "Coverage HTML:    $coverage_dir/coverage.html"
        return 0
    fi

    if command -v lcov >/dev/null 2>&1 && command -v genhtml >/dev/null 2>&1; then
        echo "--- Generating coverage report with lcov/genhtml ---"
        lcov --capture \
            --directory "$BUILD_DIR" \
            --base-directory "$EMU_PC_DIR" \
            --output-file "$coverage_dir/coverage.info" \
            --rc lcov_branch_coverage=1 >/dev/null

        lcov --remove "$coverage_dir/coverage.info" \
            "$EMU_PC_DIR/tests/*" \
            "*/_deps/*" \
            "$EMU_PC_DIR/build-*/*" \
            "/usr/*" \
            --output-file "$coverage_dir/coverage.filtered.info" \
            --rc lcov_branch_coverage=1 >/dev/null

        genhtml "$coverage_dir/coverage.filtered.info" \
            --branch-coverage \
            --output-directory "$coverage_dir/html" >/dev/null

        echo "Coverage info: $coverage_dir/coverage.filtered.info"
        echo "Coverage HTML: $coverage_dir/html/index.html"
        return 0
    fi

    echo "Coverage tools not found: install gcovr or lcov+genhtml to generate reports." >&2
    return 1
}

print_help() {
        cat <<EOF
Usage: ./run_tests.sh [OPTIONS]

Run all PC-runnable unit tests for the Gaspetto project.

Options:
    --rebuild   Force CMake reconfigure + rebuild before running tests
    --clean     Remove build directory then reconfigure + rebuild
    --coverage  Use a coverage-instrumented build and generate a coverage report
    --help      Show this help message and exit

Examples:
    ./run_tests.sh
    ./run_tests.sh --rebuild
    ./run_tests.sh --clean
    ./run_tests.sh --coverage
EOF
}

REBUILD=0
CLEAN=0
COVERAGE=0
for arg in "$@"; do
    case "$arg" in
        --rebuild)
            REBUILD=1
            ;;
        --clean)
            CLEAN=1
            ;;
        --coverage)
            COVERAGE=1
            ;;
        --help|-h)
            print_help
            exit 0
            ;;
        *)
            echo "Unknown option: $arg" >&2
            echo "Use --help to see available options." >&2
            exit 1
            ;;
    esac
done

if [[ $COVERAGE -eq 1 ]]; then
    BUILD_DIR="$COVERAGE_BUILD_DIR"
fi

echo "=== Gaspetto Unit Tests ==="
echo "Source: $EMU_PC_DIR"
echo "Build:  $BUILD_DIR"
echo ""

# Clean build directory if requested
if [[ $CLEAN -eq 1 && -d "$BUILD_DIR" ]]; then
    echo "--- Cleaning $BUILD_DIR ---"
    rm -rf "$BUILD_DIR"
    echo ""
    REBUILD=1
fi

# Reconfigure if requested or build directory doesn't exist
if [[ $REBUILD -eq 1 || ! -d "$BUILD_DIR" ]]; then
    if [[ $COVERAGE -eq 1 ]]; then
        echo "--- Configuring coverage build ---"
        cmake -S "$EMU_PC_DIR" -B "$BUILD_DIR" -G Ninja \
            -DCMAKE_BUILD_TYPE=Debug \
            -DBUILD_TESTS=ON \
            -DCMAKE_C_FLAGS="$COVERAGE_FLAGS" \
            -DCMAKE_CXX_FLAGS="$COVERAGE_FLAGS" \
            -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
            -DCMAKE_SHARED_LINKER_FLAGS="--coverage"
    else
        echo "--- Configuring (cmake --preset tests) ---"
        cmake --preset tests -S "$EMU_PC_DIR" -B "$BUILD_DIR"
    fi
    echo ""
fi

echo "--- Building test targets ---"
cmake --build "$BUILD_DIR" --parallel "$NUM_CORES" --target utest_box utest_car
echo ""

echo "=== BOX TESTS ==="
"$BUILD_DIR/tests/utest_box" --gtest_color=yes
echo ""

echo "=== CAR TESTS ==="
"$BUILD_DIR/tests/utest_car" --gtest_color=yes

if [[ $COVERAGE -eq 1 ]]; then
    echo ""
    generate_coverage_report
fi

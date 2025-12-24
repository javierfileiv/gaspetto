# ====================================================================================
# Gaspetto PC Emulation - Makefile Wrapper
# ====================================================================================
# Provides easy-to-use targets wrapping the modern CMake build system
#
# Usage:
#   make              # Build car in release mode
#   make debug        # Build car in debug mode
#   make tests         # Run unit tests
#   make clean        # Clean all build artifacts
#   make tidy         # Run clang-tidy static analysis
#   make coverage     # Generate code coverage report
#   make help         # Show all available targets

.PHONY: all build debug release test tests clean tidy coverage help all_tests

# Default configuration
BUILD_DIR ?= build-car
BUILD_TYPE ?= Debug
JOBS ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Colors for output
CYAN := \033[0;36m
GREEN := \033[0;32m
YELLOW := \033[0;33m
RED := \033[0;31m
RESET := \033[0m

# ====================================================================================
# Primary Build Targets
# ====================================================================================

all: car box nrf

all_tests: tests test-sanitizers coverage

build: release

debug:
	@echo "$(CYAN)Building Gaspetto Car (Debug)...$(RESET)"
	@cmake --preset debug
	@cmake --build build-debug -j$(JOBS)
	@echo "$(GREEN)✓ Debug build complete$(RESET)"

release:
	@echo "$(CYAN)Building Gaspetto Car (Release)...$(RESET)"
	@cmake --preset release
	@cmake --build build-release -j$(JOBS)
	@echo "$(GREEN)✓ Release build complete$(RESET)"

# ====================================================================================
# Testing Targets
# ====================================================================================

tests:
	@echo "$(CYAN)Building and running tests...$(RESET)"
	@mkdir -p build-tests
	@cd build-tests && cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON \
		-DBUILD_GASPETTO_CAR=OFF -DBUILD_GASPETTO_BOX=OFF -DBUILD_NRF_SENDER=OFF
	@cmake --build build-tests -j$(JOBS)
	@echo "$(CYAN)Running unit tests...$(RESET)"
	@cd build-tests && ./bin/utest
	@echo "$(GREEN)✓ All tests passed$(RESET)"

test-sanitizers:
	@echo "$(CYAN)Building tests with sanitizers...$(RESET)"
	@cmake --preset tests-sanitizers
	@cmake --build build-sanitizers -j$(JOBS)
	@echo "$(CYAN)Running tests with sanitizers...$(RESET)"
	@cd build-sanitizers && ./bin/utest
	@echo "$(GREEN)✓ Tests with sanitizers passed$(RESET)"

coverage:
	@echo "$(CYAN)Generating code coverage...$(RESET)"
	@mkdir -p build-coverage
	@cd build-coverage && cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON \
		-DENABLE_COVERAGE=ON \
		-DBUILD_GASPETTO_CAR=OFF -DBUILD_GASPETTO_BOX=OFF -DBUILD_NRF_SENDER=OFF
	@cmake --build build-coverage -j$(JOBS)
	@cd build-coverage && ./bin/utest
	@cd build-coverage && lcov --capture --directory . --output-file coverage.info --ignore-errors mismatch
	@cd build-coverage && lcov --remove coverage.info \
		'/usr/*' '*/_deps/*' '*/tests/*' \
		--output-file coverage.info --ignore-errors mismatch,unused
	@cd build-coverage && genhtml coverage.info --output-directory coverage --ignore-errors mismatch
	@echo "$(GREEN)✓ Coverage report generated in build-coverage/coverage/index.html$(RESET)"
	@which xdg-open > /dev/null && xdg-open build-coverage/coverage/index.html || true

# ====================================================================================
# Code Quality Targets
# ====================================================================================

tidy:
	@echo "$(CYAN)Running clang-tidy static analysis...$(RESET)"
	@cmake -B build-analysis -DCMAKE_BUILD_TYPE=Debug -DENABLE_CLANG_TIDY=ON -DBUILD_GASPETTO_CAR=ON
	@cmake --build build-analysis -j$(JOBS) 2>&1 | tee build-analysis/clang-tidy.log
	@echo "$(GREEN)✓ Static analysis complete - see build-analysis/clang-tidy.log$(RESET)"

cppcheck:
	@echo "$(CYAN)Running cppcheck...$(RESET)"
	@cmake -B build-cppcheck -DCMAKE_BUILD_TYPE=Debug -DENABLE_CPPCHECK=ON -DBUILD_GASPETTO_CAR=ON
	@cmake --build build-cppcheck -j$(JOBS)
	@echo "$(GREEN)✓ Cppcheck complete$(RESET)"

# ====================================================================================
# Build Variants
# ====================================================================================

car:
	@echo "$(CYAN)Building Gaspetto Car...$(RESET)"
	@mkdir -p build-car
	@cd build-car && cmake .. -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DBUILD_GASPETTO_CAR=ON
	@cmake --build build-car -j$(JOBS)
	@echo "$(GREEN)✓ Car build complete: build-car/bin/gaspetto_car$(RESET)"

box:
	@echo "$(CYAN)Building Gaspetto Box...$(RESET)"
	@mkdir -p build-box
	@cd build-box && cmake .. -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DBUILD_GASPETTO_BOX=ON
	@cmake --build build-box -j$(JOBS)
	@echo "$(GREEN)✓ Box build complete: build-box/bin/gaspetto_box$(RESET)"

nrf:
	@echo "$(CYAN)Building NRF Sender...$(RESET)"
	@mkdir -p build-nrf
	@cd build-nrf && cmake .. -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DBUILD_NRF_SENDER=ON
	@cmake --build build-nrf -j$(JOBS)
	@echo "$(GREEN)✓ NRF build complete: build-nrf/bin/nrf_sender$(RESET)"

# ====================================================================================
# Cleanup
# ====================================================================================

clean:
	@echo "$(YELLOW)Cleaning build artifacts...$(RESET)"
	@rm -rf build-* bin lib *.o *.a
	@echo "$(GREEN)✓ Clean complete$(RESET)"

# ====================================================================================
# Help
# ====================================================================================

help:
	@echo "$(CYAN)==========================================================================$(RESET)"
	@echo "$(CYAN)Gaspetto PC Emulation - Build System$(RESET)"
	@echo "$(CYAN)===========================================================================$(RESET)"
	@echo ""
	@echo "$(GREEN)Main Targets:$(RESET)"
	@echo "  make              - Build car (release mode, default)"
	@echo "  make debug        - Build car (debug mode)"
	@echo "  make release      - Build car (release mode)"
	@echo "  make tests        - Build and run unit tests"
	@echo ""
	@echo "$(GREEN)Specific Targets:$(RESET)"
	@echo "  make car          - Build Gaspetto Car"
	@echo "  make box          - Build Gaspetto Box"
	@echo "  make nrf          - Build NRF Sender"
	@echo ""
	@echo "$(GREEN)Quality & Testing:$(RESET)"
	@echo "  make tests           - Run unit tests"
	@echo "  make test-sanitizers - Run tests with ASan/UBSan"
	@echo "  make coverage        - Generate code coverage report"
	@echo "  make tidy            - Run clang-tidy static analysis"
	@echo "  make cppcheck        - Run cppcheck static analysis"
	@echo ""
	@echo "$(GREEN)Maintenance:$(RESET)"
	@echo "  make clean        - Remove build artifacts"
	@echo ""
	@echo "$(GREEN)Variables:$(RESET)"
	@echo "  BUILD_TYPE={Debug|Release}  - Build configuration (default: Release)"
	@echo "  JOBS=N            - Parallel build jobs (default: $(JOBS))"
	@echo ""
	@echo "$(GREEN)Examples:$(RESET)"
	@echo "  make BUILD_TYPE=Debug JOBS=8    - Debug build with 8 jobs"
	@echo "  make test         - Run all tests"
	@echo "  make coverage     - Generate coverage report"
	@echo "$(CYAN)===========================================================================$(RESET)"

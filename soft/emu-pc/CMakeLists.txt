# Gaspetto PC Emulation - Modern CMake Build System

cmake_minimum_required(VERSION 3.16)

# ==============================================================================
# Project Setup
# ==============================================================================

project(
  GaspettoEmuPC
  VERSION 1.0.0
  DESCRIPTION "Gaspetto Robot Car PC Emulation & Testing Framework"
  HOMEPAGE_URL "https://github.com/yourusername/gaspetto"
  LANGUAGES CXX
)

# Prevent in-source builds
if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds are not allowed. "
      "Please create a separate build directory and run cmake from there:\n"
      "  mkdir build && cd build && cmake .."
  )
endif()

# ==============================================================================
# CMake Modules
# ==============================================================================

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Include custom CMake modules
include(Cache)
include(CompilerWarnings)
include(Sanitizers)
include(StaticAnalyzers)
include(CompilerOptimizations)
include(PrecompiledHeaders)
include(Doxygen)

# Standard CMake modules
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

# ==============================================================================
# C++ Standard & Global Settings
# ==============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Position Independent Code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ==============================================================================
# Build Options
# ==============================================================================

# Target Selection
option(BUILD_GASPETTO_CAR "Build Gaspetto Car target" ON)
option(BUILD_GASPETTO_BOX "Build Gaspetto Box target" OFF)
option(BUILD_NRF_SENDER "Build NRF Sender target" OFF)
option(BUILD_TESTS "Build unit tests" OFF)

# Quality & Analysis
option(ENABLE_WARNINGS "Enable comprehensive compiler warnings" ON)
option(TREAT_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)
option(ENABLE_SANITIZER_ADDRESS "Enable AddressSanitizer" OFF)
option(ENABLE_SANITIZER_UNDEFINED "Enable UndefinedBehaviorSanitizer" OFF)
option(ENABLE_SANITIZER_THREAD "Enable ThreadSanitizer" OFF)
option(ENABLE_COVERAGE "Enable code coverage instrumentation" OFF)

# Performance
option(ENABLE_NATIVE_ARCH "Enable -march=native optimization" ON)
option(ENABLE_UNITY_BUILD "Enable unity builds for faster compilation" OFF)
option(ENABLE_PCH "Enable precompiled headers" ON)

# Documentation & Tools
option(BUILD_DOCUMENTATION "Generate API documentation" OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
option(ENABLE_CPPCHECK "Enable cppcheck static analysis" OFF)

# Installation
option(INSTALL_HEADERS "Install development headers" ON)

# ==============================================================================
# Compiler & Linker Configuration
# ==============================================================================

# Create interface library for common project settings
add_library(project_options INTERFACE)
add_library(project_warnings INTERFACE)

# Apply compiler warnings
if(ENABLE_WARNINGS)
  set_project_warnings(project_warnings)
endif()

# Apply sanitizers
enable_sanitizers(project_options)

# Also apply sanitizers globally for third-party dependencies like GoogleTest
if(ENABLE_SANITIZER_ADDRESS
   OR ENABLE_SANITIZER_UNDEFINED
   OR ENABLE_SANITIZER_THREAD
   OR ENABLE_SANITIZER_MEMORY
   OR ENABLE_SANITIZER_LEAK
)
  set(SANITIZER_FLAGS "")
  if(ENABLE_SANITIZER_ADDRESS)
    list(APPEND SANITIZER_FLAGS "address")
  endif()
  if(ENABLE_SANITIZER_UNDEFINED)
    list(APPEND SANITIZER_FLAGS "undefined")
  endif()
  if(ENABLE_SANITIZER_THREAD)
    list(APPEND SANITIZER_FLAGS "thread")
  endif()
  if(ENABLE_SANITIZER_MEMORY)
    list(APPEND SANITIZER_FLAGS "memory")
  endif()
  if(ENABLE_SANITIZER_LEAK)
    list(APPEND SANITIZER_FLAGS "leak")
  endif()

  list(JOIN SANITIZER_FLAGS "," SANITIZER_LIST)
  add_compile_options(-fsanitize=${SANITIZER_LIST} -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${SANITIZER_LIST})
endif()

# Common compile definitions
target_compile_definitions(
  project_options INTERFACE USE_RADIO_CONTROLLER LOW_POWER_MODE GASPETTO_LOG NRF_LOG
                            $<$<CONFIG:Debug>:DEBUG_BUILD> $<$<CONFIG:Release>:NDEBUG>
)

# Build type specific flags
target_compile_options(
  project_options
  INTERFACE $<$<CONFIG:Debug>:-g
            -O0>
            $<$<CONFIG:Release>:-O3>
            $<$<CONFIG:RelWithDebInfo>:-O2
            -g>
            $<$<CONFIG:MinSizeRel>:-Os>
            -pipe
)

# Code coverage
if(ENABLE_COVERAGE AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(project_options INTERFACE --coverage)
  target_link_options(project_options INTERFACE --coverage)
  # Apply coverage flags globally so FetchContent dependencies inherit them
  add_compile_options(--coverage)
  add_link_options(--coverage)
  message(STATUS "Code coverage enabled")
endif()

# ==============================================================================
# Dependencies
# ==============================================================================

find_package(Threads REQUIRED)

# ==============================================================================
# Subdirectories
# ==============================================================================

# Build all framework components
add_subdirectory(state_machine_framework)
add_subdirectory(arduino_framework)
add_subdirectory(radio_controller)
# add_subdirectory(read_hall)  # Optional - Currently has API mismatch

# Target-specific builds
if(BUILD_GASPETTO_CAR OR BUILD_TESTS)
  set(GASPETTO_CAR ON)
  add_subdirectory(targets/gaspetto_car)
endif()

if(BUILD_GASPETTO_BOX)
  set(GASPETTO_BOX ON)
  add_subdirectory(targets/gaspetto_box)
endif()

if(BUILD_NRF_SENDER)
  set(NRF_SENDER ON)
  add_subdirectory(targets/nrf_sender)
endif()

# Unit tests
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ==============================================================================
# Installation
# ==============================================================================

if(INSTALL_HEADERS)
  install(
    DIRECTORY arduino_framework/include/ state_machine_framework/include/ radio_controller/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gaspetto
    FILES_MATCHING
    PATTERN "*.h"
  )
endif()

# Install executables
if(BUILD_GASPETTO_CAR AND NOT BUILD_TESTS)
  install(TARGETS gaspetto_car RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(BUILD_GASPETTO_BOX)
  install(TARGETS gaspetto_box RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

if(BUILD_NRF_SENDER)
  install(TARGETS nrf_sender RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

# ==============================================================================
# Build Summary
# ==============================================================================

message(STATUS "")
message(STATUS "========================================")
message(STATUS "  ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "========================================")
message(STATUS "")
message(STATUS "Build Configuration:")
message(STATUS "  CMake Version:      ${CMAKE_VERSION}")
message(STATUS "  Build Type:         ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:       ${CMAKE_CXX_STANDARD}")
message(STATUS "  Compiler:           ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "")
message(STATUS "Build Targets:")
message(STATUS "  Gaspetto Car:       ${BUILD_GASPETTO_CAR}")
message(STATUS "  Gaspetto Box:       ${BUILD_GASPETTO_BOX}")
message(STATUS "  NRF Sender:         ${BUILD_NRF_SENDER}")
message(STATUS "  Unit Tests:         ${BUILD_TESTS}")
message(STATUS "")
message(STATUS "Quality & Analysis:")
message(STATUS "  Warnings:           ${ENABLE_WARNINGS}")
message(STATUS "  Warnings as Errors: ${TREAT_WARNINGS_AS_ERRORS}")
message(STATUS "  Address Sanitizer:  ${ENABLE_SANITIZER_ADDRESS}")
message(STATUS "  UB Sanitizer:       ${ENABLE_SANITIZER_UNDEFINED}")
message(STATUS "  Thread Sanitizer:   ${ENABLE_SANITIZER_THREAD}")
message(STATUS "  Code Coverage:      ${ENABLE_COVERAGE}")
message(STATUS "  clang-tidy:         ${ENABLE_CLANG_TIDY}")
message(STATUS "  cppcheck:           ${ENABLE_CPPCHECK}")
message(STATUS "")
message(STATUS "Performance:")
message(STATUS "  Native Arch:        ${ENABLE_NATIVE_ARCH}")
message(STATUS "  Unity Build:        ${ENABLE_UNITY_BUILD}")
message(STATUS "  Precompiled Headers:${ENABLE_PCH}")
message(STATUS "  Build Cache:        ${ENABLE_CACHE}")
if(CCACHE_FOUND)
  message(STATUS "    Cache Tool:       ccache")
elseif(SCCACHE_FOUND)
  message(STATUS "    Cache Tool:       sccache")
endif()
message(STATUS "")
message(STATUS "Installation:")
message(STATUS "  Prefix:             ${CMAKE_INSTALL_PREFIX}")
message(STATUS "  Install Headers:    ${INSTALL_HEADERS}")
message(STATUS "")
message(STATUS "========================================")
message(STATUS "")

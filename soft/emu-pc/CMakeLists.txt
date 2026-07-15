cmake_minimum_required(VERSION 3.10)
project(GaspettoEmuPC VERSION 1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable debug symbols and optimization flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -pipe -march=native -flto")

# Generate compile_commands.json for IDE tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find ccache if available
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  message(STATUS "Using ccache for faster builds")
endif()

# Project options for conditional builds
option(GCAR "Build Gaspetto Car target" OFF)
option(GBOX "Build Gaspetto Box target" OFF)
option(NRF_SENDER "Build NRF Sender target" OFF)
option(BUILD_TESTS "Build unit tests" OFF)

# Define global compile definitions before adding subdirectories
add_compile_definitions(USE_RADIO_CONTROLLER LOW_POWER_MODE GASPETTO_LOG NRF_LOG)

# Dependencies must be added in order: state_machine_framework first (provides generic templates)
# shared_types next (provides application-specific Event, StateId) then radio_controller and
# arduino_framework
add_subdirectory(state_machine_framework)
add_subdirectory(shared_types)
add_subdirectory(radio_controller)
add_subdirectory(arduino_framework)

if(GCAR OR BUILD_TESTS)
  add_subdirectory(${CMAKE_SOURCE_DIR}/targets/gcar)
endif()
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
if(GBOX OR BUILD_TESTS)
  add_subdirectory(targets/gbox)
endif()
if(NRF_SENDER)
  add_subdirectory(targets/nrf_sender)
endif()

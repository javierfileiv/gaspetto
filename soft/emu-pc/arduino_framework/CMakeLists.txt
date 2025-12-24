# ==============================================================================
# Arduino Framework - PC Emulation Layer
# ==============================================================================

# Interface library for headers only
add_library(arduino_framework_itf INTERFACE)
target_include_directories(arduino_framework_itf INTERFACE include RF24 HardwareTimer)

# Implementation library
add_library(arduino_framework_impl STATIC Serial.cpp)

# Conditional compilation based on build target
if(NOT BUILD_TESTS)
  # Include hardware emulation (not needed for tests with mocks)
  target_sources(
    arduino_framework_impl PRIVATE RF24/RF24.cpp Arduino.cpp HardwareTimer/HardwareTimer.cpp
  )
endif()

# Target-specific implementations
if(GASPETTO_CAR)
  target_compile_definitions(arduino_framework_impl PRIVATE GASPETTO_CAR)
  target_sources(arduino_framework_impl PRIVATE implementations/src/gaspetto_car.cpp)
elseif(GASPETTO_BOX)
  target_compile_definitions(arduino_framework_impl PRIVATE GASPETTO_BOX)
  target_sources(arduino_framework_impl PRIVATE implementations/src/gaspetto_box.cpp)
elseif(NRF_SENDER)
  target_compile_definitions(arduino_framework_impl PRIVATE NRF_SENDER)
  target_sources(arduino_framework_impl PRIVATE implementations/src/nrf_sender.cpp)
endif()

# Main entry point
if(NOT BUILD_TESTS)
  target_sources(arduino_framework_impl PRIVATE main.cpp)
endif()

# Include paths
target_include_directories(arduino_framework_impl PRIVATE implementations/include)

# Link dependencies
target_link_libraries(
  arduino_framework_impl
  PUBLIC arduino_framework_itf
  PRIVATE state_machine_framework_itf Threads::Threads project_options project_warnings
)

# Apply optimizations
set_optimization_flags(arduino_framework_impl)

# Precompiled headers
add_project_pch(arduino_framework_impl)

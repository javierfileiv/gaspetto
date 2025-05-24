cmake_minimum_required(VERSION 3.10)

project(utest)

enable_testing()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Download and add GoogleTest if not present.
include(FetchContent)
FetchContent_Declare(
  googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                 DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(googletest)

# Add tests/ as an include directory for all relevant targets for test builds.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Workaround: explicitly add the build path for gaspetto_car_impl.
link_directories(${CMAKE_BINARY_DIR}/targets/gaspetto_car)

add_executable(
  utest
  main.cpp
  gaspetto_car/test_motor_controller.cpp
  gaspetto_car/test_radio_controller.cpp
  mock_arduino.cpp
  mock_MotorController.cpp
  mock_RadioController.cpp
  mock_RF24.cpp
  stub_RF24.cpp
  fixture.cpp
)

target_include_directories(utest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Remove linkage to arduino_framework (implementation lib) from utest. Only link
# arduino_framework_itf (interface lib).
target_link_libraries(
  utest
  gtest
  gmock
  pthread
  gaspetto_car_impl
  state_machine_framework
  arduino_framework_itf
  motor_controller
  radio_controller
)

# Enable parallel LTO for faster linking.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(utest PRIVATE -flto=auto)
  target_link_options(utest PRIVATE -flto=auto)
endif()

# Also add tests/ include dir to dependencies so stub Arduino.h is found.
foreach(dep gaspetto_car_impl state_machine_framework motor_controller)
  if(TARGET ${dep})
    target_include_directories(${dep} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
endforeach()

# Add mock_RF24.cpp to test sources.
set_property(
  TARGET utest
  APPEND
  PROPERTY SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/mock_RF24.cpp
)

add_test(NAME utest COMMAND utest)

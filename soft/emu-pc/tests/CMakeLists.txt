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

add_executable(
  utest
  main.cpp
  gaspetto_car/test_motor_controller.cpp
  gaspetto_car/test_radio_controller.cpp
  mocks/mock_arduino.cpp
  mocks/mock_MotorController.cpp
  mocks/mock_RadioController.cpp
  mocks/mock_RF24.cpp
  mocks/stub_RF24.cpp
  fixture.cpp
)
target_include_directories(utest PRIVATE ./ mocks)

target_link_libraries(
  utest
  gtest
  gmock
  pthread
  gaspetto_car_impl
  arduino_framework_impl
  state_machine_framework_impl
  motor_controller_impl
  radio_controller_impl
)

# # Enable parallel LTO for faster linking. if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
# target_compile_options(utest PRIVATE -flto=auto) target_link_options(utest PRIVATE -flto=auto)
# endif()

# # Add mock_RF24.cpp to test sources. set_property( TARGET utest APPEND PROPERTY SOURCES
# ${CMAKE_CURRENT_SOURCE_DIR}/mocks/mock_RF24.cpp )

add_test(NAME utest COMMAND utest)

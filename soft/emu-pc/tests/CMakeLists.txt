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
  utest main.cpp gaspetto_car/test_radio_controller.cpp mocks/mock_arduino.cpp mocks/mock_RF24.cpp
        mocks/stub_RF24.cpp fixture.cpp
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
  movement_controller_impl
  radio_controller_impl
)

add_test(NAME utest COMMAND utest)

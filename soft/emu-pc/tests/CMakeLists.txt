# Unit tests for Gaspetto
enable_testing()

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
  gaspetto_car/test_movement_controller.cpp
  gaspetto_car/test_radio_controller.cpp
  mocks/mock_arduino.cpp
  mocks/mock_RF24.cpp
  mocks/stub_RF24.cpp
  mocks/mock_MovementController.cpp
  fixture.cpp
)
target_include_directories(utest PRIVATE ./ mocks)

target_link_libraries(
  utest
  gtest
  gmock
  pthread
  gaspetto_car_impl
  gaspetto_car_itf
  arduino_framework_itf
  arduino_framework_impl
  state_machine_framework_itf
  movement_controller_itf
  radio_controller_impl
  shared_types
)

add_test(NAME utest COMMAND utest)

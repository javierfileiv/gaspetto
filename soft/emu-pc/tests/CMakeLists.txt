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
  utest_car
  main.cpp
  gaspetto_car/test_movement_controller.cpp
  gaspetto_car/test_radio_controller.cpp
  gaspetto_car/test_radio_controller_unit.cpp
  gaspetto_car/test_pid_movement.cpp
  gaspetto_car/test_system_pid.cpp
  gaspetto_car/test_motor_imu.cpp
  mocks/mock_arduino.cpp
  mocks/mock_RF24.cpp
  mocks/stub_RF24.cpp
  mocks/mock_MovementController.cpp
  mocks/mock_IMUOrientation.cpp
  mocks/mock_MotorControl.cpp
  fixture.cpp
)
target_include_directories(utest_car PRIVATE ./ mocks)

target_link_libraries(
  utest_car
  gtest
  gmock
  pthread
  gaspetto_car_impl
  gaspetto_car_itf
  arduino_framework_itf
  arduino_framework_impl
  state_machine_framework_itf
  movement_controller_itf
  movement_controller_impl
  radio_controller_impl
  shared_types
)

add_test(NAME utest_car COMMAND utest_car)

add_executable(
  utest_box main.cpp gaspetto_box/test_program_builder.cpp gaspetto_box/test_fsm.cpp
            mocks/mock_RF24.cpp
)
target_include_directories(utest_box PRIVATE ./ mocks)
target_link_libraries(
  utest_box
  gtest
  gmock
  pthread
  gaspetto_box_impl
  gaspetto_box_itf
  arduino_framework_itf
  arduino_framework_impl
  radio_controller_impl
  state_machine_framework_itf
  shared_types
)

add_test(NAME utest_box COMMAND utest_box)

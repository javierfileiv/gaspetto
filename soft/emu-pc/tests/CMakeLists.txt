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
  gcar/test_movement_controller.cpp
  gcar/test_radio_controller.cpp
  gcar/test_radio_controller_unit.cpp
  gcar/test_pid_movement.cpp
  gcar/test_system_pid.cpp
  gcar/test_motor_imu.cpp
  gcar/test_queue_clear.cpp
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
  gcar_impl
  gcar_itf
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
  utest_box main.cpp gbox/test_program_builder.cpp gbox/test_fsm.cpp gbox/test_button_interrupt.cpp
            mocks/mock_RF24.cpp
)
target_include_directories(utest_box PRIVATE ./ mocks)
target_link_libraries(
  utest_box
  gtest
  gmock
  pthread
  gbox_impl
  gbox_itf
  arduino_framework_itf
  arduino_framework_impl
  radio_controller_impl
  state_machine_framework_itf
  shared_types
)

add_test(NAME utest_box COMMAND utest_box)

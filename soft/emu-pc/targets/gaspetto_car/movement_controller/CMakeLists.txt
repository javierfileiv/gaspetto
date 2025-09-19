add_library(movement_controller_itf INTERFACE)
target_include_directories(movement_controller_itf INTERFACE include ./)

add_library(movement_controller_impl STATIC src/MovementController.cpp src/IMUOrientation.cpp)

if(NOT BUILD_TESTS)
  target_sources(movement_controller_impl PRIVATE src/MotorControl.cpp)
endif()

target_link_libraries(
  movement_controller_impl
  PUBLIC movement_controller_itf
  PRIVATE arduino_framework_impl state_machine_framework_itf
)

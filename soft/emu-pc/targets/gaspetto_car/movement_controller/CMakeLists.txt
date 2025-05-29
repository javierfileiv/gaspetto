add_library(movement_controller_itf INTERFACE)
target_include_directories(movement_controller_itf INTERFACE include ./)

add_library(movement_controller_impl STATIC src/MotorControl.cpp src/MovementController.cpp)
target_link_libraries(
  movement_controller_impl
  PUBLIC movement_controller_itf
  PRIVATE arduino_framework_itf state_machine_framework_itf
)

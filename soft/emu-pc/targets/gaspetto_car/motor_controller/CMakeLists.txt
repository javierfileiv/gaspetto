add_library(motor_controller_itf INTERFACE)
target_include_directories(motor_controller_itf INTERFACE include)

add_library(motor_controller_impl STATIC src/MotorController.cpp)
target_link_libraries(
  motor_controller_impl
  PUBLIC motor_controller_itf
  PRIVATE arduino_framework_itf state_machine_framework_itf
)

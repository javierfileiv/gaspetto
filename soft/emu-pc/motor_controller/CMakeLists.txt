add_library(motor_controller STATIC src/MotorController.cpp)

add_library(motor_controller_itf INTERFACE)

target_include_directories(motor_controller_itf INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
  motor_controller
  PUBLIC motor_controller_itf
  PRIVATE arduino_framework_impl state_machine_framework
)

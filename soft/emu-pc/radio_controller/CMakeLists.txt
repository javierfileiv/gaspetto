add_library(radio_controller_impl STATIC src/RadioController.cpp)

add_library(radio_controller_itf INTERFACE)
target_include_directories(radio_controller_itf INTERFACE include)

target_link_libraries(
  radio_controller_impl
  PUBLIC radio_controller_itf arduino_framework_itf
  PRIVATE state_machine_framework_itf
)

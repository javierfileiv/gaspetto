add_library(radio_controller STATIC src/RadioController.cpp)

add_library(radio_controller_itf INTERFACE)

target_include_directories(radio_controller_itf INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
  radio_controller
  PUBLIC radio_controller_itf
  PRIVATE arduino_framework_itf state_machine_framework
)

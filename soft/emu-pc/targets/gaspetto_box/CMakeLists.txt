set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(
  gaspetto_box src/main.cpp src/GaspettoBox.cpp src/IdleState.cpp src/ProcessingState.cpp
)

target_include_directories(gaspetto_box PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
  gaspetto_box PRIVATE arduino_framework_impl state_machine_framework_impl radio_controller_impl
)

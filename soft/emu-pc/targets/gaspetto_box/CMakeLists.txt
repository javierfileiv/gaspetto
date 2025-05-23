set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(
  gaspetto_box src/main.cpp src/GaspettoBox.cpp src/IdleState.cpp src/ProcessingState.cpp
)

target_include_directories(gaspetto_box PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
  gaspetto_box PRIVATE arduino_framework state_machine_framework radio_controller
)

target_compile_definitions(gaspetto_box PRIVATE GASPETTO_BOX)

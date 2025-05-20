add_executable(
  gaspetto_car src/main.cpp src/GaspettoCar.cpp src/IdleState.cpp src/ProcessingState.cpp
)

target_include_directories(
  gaspetto_car PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                       ${CMAKE_CURRENT_SOURCE_DIR}/include/emu_pc
)

target_link_libraries(
  gaspetto_car PRIVATE arduino_framework state_machine_framework motor_controller radio_controller
)

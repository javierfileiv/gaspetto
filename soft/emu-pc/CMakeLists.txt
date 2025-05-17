cmake_minimum_required(VERSION 3.10)
project(GaspettoEmuPC)

# Enable generation of compile_commands.json for clang tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(arduino_framework)
add_subdirectory(state_machine_framework)
add_subdirectory(gaspetto_car)
add_subdirectory(gaspetto_box)

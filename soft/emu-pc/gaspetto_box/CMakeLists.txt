add_executable(gaspetto_box main.cpp ProcessingState.cpp IdleState.cpp GaspettoBox.cpp)

target_include_directories(gaspetto_box PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gaspetto_box arduino_framework state_machine_framework)
target_compile_definitions(gaspetto_box PRIVATE LOW_POWER_MODE=1 DEBUG_GASPETTO=1)

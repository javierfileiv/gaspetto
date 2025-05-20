add_library(state_machine_framework STATIC src/EventQueue.cpp src/TimeredEventQueue.cpp)

target_include_directories(state_machine_framework PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(state_machine_framework PUBLIC arduino_framework)

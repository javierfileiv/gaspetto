add_library(arduino_framework Arduino.cpp RF24.cpp)

target_include_directories(arduino_framework PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(arduino_framework PRIVATE state_machine_framework)

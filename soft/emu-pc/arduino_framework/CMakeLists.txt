add_library(arduino_framework STATIC Arduino.cpp RF24/RF24.cpp)

target_link_libraries(arduino_framework PUBLIC common_headers radio_controller)

target_include_directories(
  arduino_framework PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/RF24
)

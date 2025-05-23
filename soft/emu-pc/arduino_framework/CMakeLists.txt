add_library(arduino_framework_impl STATIC Arduino.cpp)

target_include_directories(
  arduino_framework_impl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/RF24
)
target_link_libraries(arduino_framework_impl PUBLIC common_headers radio_controller)

target_sources(
  arduino_framework_impl PRIVATE implementations/src/gaspetto_car.cpp
                                 implementations/src/nrf_sender.cpp
)

add_library(
  arduino_framework STATIC RF24/RF24.cpp Serial.cpp implementations/src/gaspetto_car.cpp
                           implementations/src/nrf_sender.cpp
)

# Set a variable to indicate the current build target for arduino_framework
if(GASPETTO_CAR)
  target_compile_definitions(arduino_framework PUBLIC GASPETTO_CAR)
elseif(GASPETTO_BOX)
  target_compile_definitions(arduino_framework PUBLIC GASPETTO_BOX)
  target_sources(arduino_framework PRIVATE implementations/src/gaspetto_box.cpp)
elseif(NRF_SENDER)
  target_compile_definitions(arduino_framework PUBLIC NRF_SENDER)
  target_sources(arduino_framework PRIVATE implementations/src/nrf_sender.cpp)
endif()

if(NOT BUILD_TESTS)
  target_sources(arduino_framework PRIVATE main.cpp)
endif()
target_include_directories(
  arduino_framework PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/implementations/include
)

target_link_libraries(
  arduino_framework PUBLIC arduino_framework_impl common_headers radio_controller
)

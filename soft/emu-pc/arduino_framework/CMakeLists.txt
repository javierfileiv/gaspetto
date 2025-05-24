add_library(arduino_framework_itf INTERFACE)

target_include_directories(
  arduino_framework_itf INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/RF24
)

add_library(arduino_framework_impl STATIC Arduino.cpp)
target_link_libraries(
  arduino_framework_impl PUBLIC state_machine_itf arduino_framework_itf radio_controller
)

target_sources(
  arduino_framework_impl
  PRIVATE implementations/src/gaspetto_car.cpp implementations/src/gaspetto_box.cpp
          implementations/src/nrf_sender.cpp
)

add_library(
  arduino_framework STATIC Serial.cpp implementations/src/gaspetto_car.cpp
                           implementations/src/nrf_sender.cpp
)

# Do not build emulated RF24 if testing, like that we use the mocked one.
if(NOT BUILD_TESTS)
  target_sources(arduino_framework PRIVATE RF24/RF24.cpp)
endif()

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
  arduino_framework PUBLIC arduino_framework_impl state_machine_itf radio_controller
)

add_library(arduino_framework_itf INTERFACE)
target_include_directories(arduino_framework_itf INTERFACE include RF24 HardwareTimer)
add_library(arduino_framework_impl STATIC Serial.cpp)
# Do not build emulated RF24 or Arduino if testing, like that we use the mocked one.
if(NOT BUILD_TESTS)
  target_sources(
    arduino_framework_impl PRIVATE RF24/RF24.cpp Arduino.cpp HardwareTimer/HardwareTimer.cpp
  )
endif()
# Set a variable to indicate the current build target for arduino_framework
if(GASPETTO_CAR)
  target_compile_definitions(arduino_framework_impl PRIVATE GASPETTO_CAR)
  target_sources(arduino_framework_impl PRIVATE implementations/src/gaspetto_car.cpp)
elseif(GASPETTO_BOX)
  target_compile_definitions(arduino_framework_impl PRIVATE GASPETTO_BOX)
  target_sources(arduino_framework_impl PRIVATE implementations/src/gaspetto_box.cpp)
elseif(NRF_SENDER)
  target_compile_definitions(arduino_framework_impl PRIVATE NRF_SENDER)
  target_sources(arduino_framework_impl PRIVATE implementations/src/nrf_sender.cpp)
endif()

if(NOT BUILD_TESTS)
  target_sources(arduino_framework_impl PRIVATE main.cpp)
endif()
target_include_directories(arduino_framework_impl PRIVATE implementations/include)
target_link_libraries(arduino_framework_impl arduino_framework_itf state_machine_framework_itf)

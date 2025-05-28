set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(nrf_sender src/main.cpp)

target_compile_definitions(nrf_sender PRIVATE NRF_SENDER)

target_link_libraries(nrf_sender arduino_framework_impl radio_controller_impl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(nrf_sender src/main.cpp)

target_include_directories(
  nrf_sender PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/include/emu_pc
)

target_compile_definitions(nrf_sender PRIVATE NRF_SENDER)

target_link_libraries(nrf_sender arduino_framework radio_controller)

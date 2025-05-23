set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

message(
  STATUS "[CMake] Processing targets/gaspetto_car/CMakeLists.txt and building gaspetto_car_impl!"
)

add_library(
  gaspetto_car_impl STATIC src/GaspettoCar.cpp src/IdleState.cpp src/ProcessingState.cpp
                           include/IdleState.h include/ProcessingState.h
)

target_include_directories(
  gaspetto_car_impl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/tests
)

target_compile_definitions(gaspetto_car_impl PRIVATE GASPETTO_CAR)

target_link_libraries(
  gaspetto_car_impl PRIVATE state_machine_framework radio_controller_itf motor_controller_itf
)

add_executable(gaspetto_car src/main.cpp)

target_include_directories(
  gaspetto_car PRIVATE ${CMAKE_SOURCE_DIR}/arduino_framework ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  gaspetto_car PRIVATE gaspetto_car_impl arduino_framework state_machine_framework motor_controller
                       radio_controller
)

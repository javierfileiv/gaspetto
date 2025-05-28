set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(motor_controller)

add_library(gaspetto_car_impl STATIC src/GaspettoCar.cpp src/IdleState.cpp src/ProcessingState.cpp)
target_include_directories(gaspetto_car_impl PUBLIC include)
target_link_libraries(
  gaspetto_car_impl PRIVATE radio_controller_itf motor_controller_itf state_machine_framework_itf
)

add_executable(gaspetto_car src/main.cpp)
target_link_libraries(
  gaspetto_car PRIVATE gaspetto_car_impl motor_controller_impl arduino_framework_impl
                       state_machine_framework_impl radio_controller_impl
)

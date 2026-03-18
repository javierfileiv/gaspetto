set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(movement_controller)

# Interface library for gaspetto_car types
add_library(gaspetto_car_itf INTERFACE)
target_include_directories(gaspetto_car_itf INTERFACE include)
target_link_libraries(gaspetto_car_itf INTERFACE state_machine_framework_itf shared_types)

add_library(gaspetto_car_impl STATIC src/GaspettoCar.cpp src/IdleState.cpp src/ProcessingState.cpp)
target_include_directories(gaspetto_car_impl PUBLIC include)
target_link_libraries(
  gaspetto_car_impl PRIVATE radio_controller_itf movement_controller_itf
                            state_machine_framework_itf gaspetto_car_itf shared_types
)

add_executable(gaspetto_car src/main.cpp)
target_link_libraries(
  gaspetto_car PRIVATE gaspetto_car_impl movement_controller_impl arduino_framework_impl
                       radio_controller_impl gaspetto_car_itf shared_types
)

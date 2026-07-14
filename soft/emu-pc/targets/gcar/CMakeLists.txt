set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(movement_controller)

# Interface library for gcar types
add_library(gcar_itf INTERFACE)
target_include_directories(gcar_itf INTERFACE include)
target_link_libraries(gcar_itf INTERFACE state_machine_framework_itf shared_types)

add_library(gcar_impl STATIC src/GCar.cpp src/IdleState.cpp src/ProcessingState.cpp)
target_include_directories(gcar_impl PUBLIC include)
target_link_libraries(
  gcar_impl PRIVATE radio_controller_itf movement_controller_itf state_machine_framework_itf
                    gcar_itf shared_types
)

add_executable(gcar src/main.cpp)
target_link_libraries(
  gcar PRIVATE gcar_impl movement_controller_impl arduino_framework_impl radio_controller_impl
               gcar_itf shared_types
)

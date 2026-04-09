set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(gaspetto_box_itf INTERFACE)
target_include_directories(gaspetto_box_itf INTERFACE include)
target_link_libraries(gaspetto_box_itf INTERFACE state_machine_framework_itf shared_types)

add_library(gaspetto_box_impl STATIC src/GaspettoBox.cpp src/IdleState.cpp src/ProcessingState.cpp)
target_include_directories(gaspetto_box_impl PUBLIC include)
target_link_libraries(
  gaspetto_box_impl PRIVATE radio_controller_itf state_machine_framework_itf gaspetto_box_itf
                            shared_types
)

add_executable(gaspetto_box src/main.cpp)
target_link_libraries(
  gaspetto_box PRIVATE gaspetto_box_impl arduino_framework_impl radio_controller_impl
                       gaspetto_box_itf shared_types
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(gbox_itf INTERFACE)
target_include_directories(gbox_itf INTERFACE include)
target_link_libraries(gbox_itf INTERFACE state_machine_framework_itf shared_types)

add_library(gbox_impl STATIC src/GBox.cpp src/IdleState.cpp src/ProcessingState.cpp)
target_include_directories(gbox_impl PUBLIC include)
target_link_libraries(
  gbox_impl PRIVATE radio_controller_itf state_machine_framework_itf gbox_itf shared_types
)

add_executable(gbox src/main.cpp)
target_link_libraries(
  gbox PRIVATE gbox_impl arduino_framework_impl radio_controller_impl gbox_itf shared_types
)

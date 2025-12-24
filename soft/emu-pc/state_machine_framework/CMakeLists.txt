# ==============================================================================
# State Machine Framework - Event-Driven Architecture
# ==============================================================================

# Interface library for headers
add_library(state_machine_framework_itf INTERFACE)
target_include_directories(state_machine_framework_itf INTERFACE include)
target_link_libraries(state_machine_framework_itf INTERFACE arduino_framework_itf)

# Implementation library
add_library(state_machine_framework_impl STATIC src/EventQueue.cpp src/TimeredEventQueue.cpp)

target_link_libraries(
  state_machine_framework_impl
  PUBLIC state_machine_framework_itf
  PRIVATE project_options project_warnings
)

# Apply optimizations
set_optimization_flags(state_machine_framework_impl)

# Precompiled headers
add_project_pch(state_machine_framework_impl)

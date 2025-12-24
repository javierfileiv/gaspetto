# ==============================================================================
# Read Hall - Odometry from Hall Effect Sensors
# ==============================================================================

# Interface library
add_library(read_hall_itf INTERFACE)
target_include_directories(read_hall_itf INTERFACE include)
target_link_libraries(read_hall_itf INTERFACE arduino_framework_itf state_machine_framework_itf)

# Implementation library
add_library(read_hall_impl STATIC src/ReadHall.cpp)

target_link_libraries(
  read_hall_impl
  PUBLIC read_hall_itf
  PRIVATE project_options project_warnings
)

# Apply optimizations
set_optimization_flags(read_hall_impl)

# Precompiled headers
add_project_pch(read_hall_impl)

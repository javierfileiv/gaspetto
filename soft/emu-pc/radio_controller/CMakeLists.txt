# ==============================================================================
# Radio Controller - nRF24L01+ Wireless Communication
# ==============================================================================

# Interface library for headers
add_library(radio_controller_itf INTERFACE)
target_include_directories(radio_controller_itf INTERFACE include)
target_link_libraries(
  radio_controller_itf INTERFACE arduino_framework_itf state_machine_framework_itf
)

# Implementation library
add_library(radio_controller_impl STATIC src/RadioController.cpp)

target_link_libraries(
  radio_controller_impl
  PUBLIC radio_controller_itf
  PRIVATE project_options project_warnings
)

# Apply optimizations
set_optimization_flags(radio_controller_impl)

# Precompiled headers
add_project_pch(radio_controller_impl)

# ==============================================================================
# Gaspetto Car Target - Remote Controlled Vehicle
# ==============================================================================

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Movement controller subsystem
add_subdirectory(movement_controller)

# Car state machine implementation
add_library(gaspetto_car_impl STATIC src/GaspettoCar.cpp src/IdleState.cpp src/ProcessingState.cpp)

target_include_directories(gaspetto_car_impl PUBLIC include)

target_link_libraries(
  gaspetto_car_impl PRIVATE radio_controller_itf movement_controller_itf
                            state_machine_framework_itf project_options project_warnings
)

# Apply optimizations
set_optimization_flags(gaspetto_car_impl)

# Precompiled headers
add_project_pch(gaspetto_car_impl)

# Main executable (only when not building tests)
if(NOT BUILD_TESTS)
  add_executable(gaspetto_car src/main.cpp)

  target_link_libraries(
    gaspetto_car
    PRIVATE gaspetto_car_impl
            movement_controller_impl
            arduino_framework_impl
            state_machine_framework_impl
            radio_controller_impl
            project_options
            Threads::Threads
  )

  # Set output directory
  set_target_properties(gaspetto_car PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

  # Apply optimizations
  set_optimization_flags(gaspetto_car)
endif()

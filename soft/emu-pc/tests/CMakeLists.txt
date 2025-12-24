# ==============================================================================
# Unit Tests - GoogleTest Suite
# ==============================================================================

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# GoogleTest dependency
include(FetchContent)

# Prevent GoogleTest from overriding compiler/linker options
set(gtest_force_shared_crt # cmake-lint: disable=C0103
    ON
    CACHE BOOL "" FORCE
)

FetchContent_Declare(
  googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                 DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(googletest)

# Disable warnings for GoogleTest
if(TARGET gtest)
  target_compile_options(gtest PRIVATE -w)
endif()
if(TARGET gmock)
  target_compile_options(gmock PRIVATE -w)
endif()

# Test executable
add_executable(
  utest
  main.cpp
  fixture.cpp
  gaspetto_car/test_movement_controller.cpp
  gaspetto_car/test_radio_controller.cpp
  mocks/mock_arduino.cpp
  mocks/mock_RF24.cpp
  mocks/stub_RF24.cpp
  mocks/mock_RadioController.cpp
)

target_include_directories(
  utest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/mocks
)

target_link_libraries(
  utest
  PRIVATE GTest::gtest
          GTest::gmock
          gaspetto_car_impl
          arduino_framework_impl
          state_machine_framework_impl
          movement_controller_impl
          radio_controller_impl
          Threads::Threads
)

# Set output directory
set_target_properties(utest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(
  utest
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  PROPERTIES
  TIMEOUT 30 ENVIRONMENT "GMOCK_VERBOSE=1"
)

# Add convenience target to run tests
add_custom_target(
  run_tests
  COMMAND ${CMAKE_BINARY_DIR}/bin/utest --gtest_color=yes
  DEPENDS utest
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Running unit tests..."
)

# Add coverage target if enabled
if(ENABLE_COVERAGE)
  find_program(LCOV_PATH lcov)
  find_program(GENHTML_PATH genhtml)

  if(LCOV_PATH AND GENHTML_PATH)
    add_custom_target(
      coverage
      COMMAND ${LCOV_PATH} --directory . --zerocounters
      COMMAND ${CMAKE_BINARY_DIR}/bin/utest
      COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage.info
      COMMAND ${LCOV_PATH} --remove coverage.info '/usr/*' '*/tests/*' '*/googletest/*'
              --output-file coverage.info.cleaned
      COMMAND ${GENHTML_PATH} -o coverage_report coverage.info.cleaned
      COMMAND ${CMAKE_COMMAND} -E echo "Coverage report generated in coverage_report/index.html"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      DEPENDS utest
      COMMENT "Generating code coverage report..."
    )
  endif()
endif()

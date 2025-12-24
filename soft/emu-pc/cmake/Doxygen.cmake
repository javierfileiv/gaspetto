# ==============================================================================
# Doxygen Documentation
# ==============================================================================

option(BUILD_DOCUMENTATION "Build API documentation with Doxygen" OFF)

if(BUILD_DOCUMENTATION)
  find_package(Doxygen REQUIRED dot)

  if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # Configure Doxyfile
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # Add documentation target
    add_custom_target(
      docs
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM
    )

    message(STATUS "Doxygen documentation target 'docs' configured")
  endif()
endif()

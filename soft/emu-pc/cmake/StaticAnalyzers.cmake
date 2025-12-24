# ==============================================================================
# Static Analysis Tools Configuration
# ==============================================================================
# Integrates clang-tidy and cppcheck

option(ENABLE_CLANG_TIDY "Enable clang-tidy analysis" OFF)
option(ENABLE_CPPCHECK "Enable cppcheck analysis" OFF)

if(ENABLE_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES "clang-tidy")
  if(CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE}; -header-filter=.*;)
    message(STATUS "clang-tidy enabled")
  else()
    message(WARNING "clang-tidy requested but not found")
  endif()
endif()

if(ENABLE_CPPCHECK)
  find_program(CPPCHECK_EXE NAMES "cppcheck")
  if(CPPCHECK_EXE)
    set(CMAKE_CXX_CPPCHECK
        ${CPPCHECK_EXE}
        --enable=warning,performance,portability
        --inline-suppr
        --suppress=*:*/build/*
        --suppress=*:*/googletest/*
        --suppress=*:*/_deps/*
        --suppress=missingIncludeSystem
        --suppress=unusedFunction
        # Suppress style warnings that require significant refactoring
        --suppress=noExplicitConstructor
        --suppress=ignoredReturnValue
    )
    message(STATUS "cppcheck enabled")
  else()
    message(WARNING "cppcheck requested but not found")
  endif()
endif()

# ==============================================================================
# Compiler Warnings Configuration
# ==============================================================================
# Provides interface library with essential warning flags Using a focused set of warnings suitable
# for Arduino-style embedded codebase

# set_project_warnings(<target>) Apply project warning flags to a target. Arguments: target_name -
# The target to apply warning flags to.
function(set_project_warnings target_name)
  # Core warnings - essential for catching real bugs
  set(essential_warnings
      -Wall
      -Wextra
      -Wpedantic
      -Wnon-virtual-dtor
      -Woverloaded-virtual
      -Wnull-dereference
      -Wformat=2
      -Wimplicit-fallthrough
  )

  # GCC-specific warnings
  set(gcc_specific -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op)

  # Clang-specific warnings
  set(clang_specific -Wmost)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(project_warnings ${essential_warnings} ${gcc_specific})
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(project_warnings ${essential_warnings} ${clang_specific})
  endif()

  target_compile_options(${target_name} INTERFACE ${project_warnings})

  if(TREAT_WARNINGS_AS_ERRORS)
    target_compile_options(${target_name} INTERFACE -Werror)
  endif()
endfunction()

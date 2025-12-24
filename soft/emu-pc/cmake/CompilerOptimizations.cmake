# ==============================================================================
# Compiler Optimizations
# ==============================================================================
# Provides advanced optimization options

# set_optimization_flags(<target>) Apply optimization flags to a target. Arguments: target_name -
# The target to apply optimization flags to.
function(set_optimization_flags target_name)
  # LTO/IPO
  if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    include(CheckIPOSupported)
    check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)

    if(ipo_supported)
      set_target_properties(${target_name} PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
      message(STATUS "IPO/LTO enabled for ${target_name}")
    else()
      message(STATUS "IPO/LTO not supported for ${target_name}: ${ipo_error}")
    endif()
  endif()

  # Native architecture optimizations
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    option(ENABLE_NATIVE_ARCH "Enable -march=native optimization" ON)
    if(ENABLE_NATIVE_ARCH)
      target_compile_options(${target_name} PRIVATE -march=native -mtune=native)
    endif()
  endif()

  # Unity builds for faster compilation
  option(ENABLE_UNITY_BUILD "Enable unity builds" OFF)
  if(ENABLE_UNITY_BUILD)
    set_target_properties(${target_name} PROPERTIES UNITY_BUILD ON UNITY_BUILD_BATCH_SIZE 16)
  endif()
endfunction()

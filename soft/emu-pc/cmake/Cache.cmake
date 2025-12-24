# ==============================================================================
# Build Cache Configuration
# ==============================================================================
# Automatically detects and enables ccache or sccache

option(ENABLE_CACHE "Enable build caching" ON)

if(ENABLE_CACHE)
  set(CACHE_OPTION
      "ccache"
      CACHE STRING "Compiler cache to use"
  )
  set_property(CACHE CACHE_OPTION PROPERTY STRINGS ccache sccache)

  if(CACHE_OPTION STREQUAL "ccache")
    find_program(CCACHE_FOUND ccache)
    if(CCACHE_FOUND)
      message(STATUS "Using ccache")
      set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_FOUND})
      set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_FOUND})
    else()
      message(WARNING "ccache not found, build caching disabled")
    endif()
  elseif(CACHE_OPTION STREQUAL "sccache")
    find_program(SCCACHE_FOUND sccache)
    if(SCCACHE_FOUND)
      message(STATUS "Using sccache")
      set(CMAKE_CXX_COMPILER_LAUNCHER ${SCCACHE_FOUND})
      set(CMAKE_C_COMPILER_LAUNCHER ${SCCACHE_FOUND})
    else()
      message(WARNING "sccache not found, build caching disabled")
    endif()
  endif()
endif()

# ==============================================================================
# Sanitizers Configuration
# ==============================================================================
# Provides AddressSanitizer, UBSan, ThreadSanitizer, MemorySanitizer

# enable_sanitizers(<target>) Enable sanitizers for a target based on options. Arguments:
# target_name - The target to enable sanitizers for.
function(enable_sanitizers target_name)
  set(sanitizers "")

  option(ENABLE_SANITIZER_ADDRESS "Enable address sanitizer" FALSE)
  option(ENABLE_SANITIZER_LEAK "Enable leak sanitizer" FALSE)
  option(ENABLE_SANITIZER_UNDEFINED "Enable undefined behavior sanitizer" FALSE)
  option(ENABLE_SANITIZER_THREAD "Enable thread sanitizer" FALSE)
  option(ENABLE_SANITIZER_MEMORY "Enable memory sanitizer" FALSE)

  if(ENABLE_SANITIZER_ADDRESS)
    list(APPEND sanitizers "address")
  endif()

  if(ENABLE_SANITIZER_LEAK)
    list(APPEND sanitizers "leak")
  endif()

  if(ENABLE_SANITIZER_UNDEFINED)
    list(APPEND sanitizers "undefined")
  endif()

  if(ENABLE_SANITIZER_THREAD)
    if("address" IN_LIST sanitizers OR "leak" IN_LIST sanitizers)
      message(WARNING "Thread sanitizer cannot be used with Address or Leak sanitizer")
    else()
      list(APPEND sanitizers "thread")
    endif()
  endif()

  if(ENABLE_SANITIZER_MEMORY)
    if("address" IN_LIST sanitizers
       OR "thread" IN_LIST sanitizers
       OR "leak" IN_LIST sanitizers
    )
      message(WARNING "Memory sanitizer cannot be used with Address, Thread, or Leak sanitizer")
    else()
      list(APPEND sanitizers "memory")
    endif()
  endif()

  list(JOIN sanitizers "," sanitizer_list)

  if(sanitizer_list)
    if(NOT "${sanitizer_list}" STREQUAL "")
      target_compile_options(
        ${target_name} INTERFACE -fsanitize=${sanitizer_list} -fno-omit-frame-pointer
                                 -fno-optimize-sibling-calls
      )
      target_link_options(${target_name} INTERFACE -fsanitize=${sanitizer_list})
      message(STATUS "Enabled sanitizers: ${sanitizer_list}")
    endif()
  endif()
endfunction()

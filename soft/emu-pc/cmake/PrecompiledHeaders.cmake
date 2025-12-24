# ==============================================================================
# Precompiled Headers
# ==============================================================================
# Speeds up compilation by precompiling commonly used headers

option(ENABLE_PCH "Enable Precompiled Headers" ON)

# add_project_pch(<target>) Add precompiled headers to a target. Arguments: target_name - The target
# to add precompiled headers to.
function(add_project_pch target_name)
  if(ENABLE_PCH)
    target_precompile_headers(
      ${target_name}
      PRIVATE
      <iostream>
      <memory>
      <vector>
      <string>
      <algorithm>
      <functional>
      <chrono>
      <thread>
      <mutex>
      <condition_variable>
    )
    message(STATUS "Precompiled headers enabled for ${target_name}")
  endif()
endfunction()
